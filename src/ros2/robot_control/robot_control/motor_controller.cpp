//   GPIO27 � IN1  (lewy  kierunek A / PWM)
//   GPIO17 � IN2  (lewy  kierunek B / PWM)
//   GPIO23 � IN3  (prawy kierunek A / PWM)
//   GPIO22 � IN4  (prawy kierunek B / PWM)

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <thread>

#include <gpiod.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

static constexpr char  GPIO_CHIP[]        = "/dev/gpiochip4"; // RP1 na RPi 5
static constexpr int   PIN_LEFT_FWD       = 27;
static constexpr int   PIN_LEFT_BWD       = 17;
static constexpr int   PIN_RIGHT_FWD      = 23;
static constexpr int   PIN_RIGHT_BWD      = 22;

static constexpr double CONTROL_LOOP_HZ   = 50.0;
static constexpr double ACCEL_STEP        = 0.05;   // na tick
static constexpr double DEADZONE          = 0.01;
static constexpr int    RT_PRIORITY       = 50;     // SCHED_FIFO priorytet

class SoftPwmMotor {
public:
    SoftPwmMotor(gpiod_line * fwd, gpiod_line * bwd)
        : line_fwd_(fwd), line_bwd_(bwd) {
        running_ = true;
        pwm_thread_ = std::thread(&SoftPwmMotor::pwm_loop, this);
    }

    ~SoftPwmMotor() {
        running_ = false;
        if (pwm_thread_.joinable()) pwm_thread_.join();
        stop();
    }

    // speed ? [-1.0, +1.0]
    void drive(double speed) {
        speed = std::clamp(speed, -1.0, 1.0);
        if (std::abs(speed) < DEADZONE) speed = 0.0;
        current_speed_.store(speed);
    }

    void stop() {
        current_speed_.store(0.0);
        gpiod_line_set_value(line_fwd_, 0);
        gpiod_line_set_value(line_bwd_, 0);
    }

private:
    gpiod_line * line_fwd_;
    gpiod_line * line_bwd_;
    std::atomic<double> current_speed_{0.0};
    
    std::atomic<bool> running_{false};
    std::thread pwm_thread_;

    // P�tla generuj�ca sygna� PWM (~100 Hz)
    void pwm_loop() {
        const int freq_hz = 100;
        const auto period = std::chrono::microseconds(1000000 / freq_hz);
        
        while (running_) {
            double s = current_speed_.load();
            
            if (s == 0.0) {
                gpiod_line_set_value(line_fwd_, 0);
                gpiod_line_set_value(line_bwd_, 0);
                std::this_thread::sleep_for(period);
                continue;
            }

            bool forward = (s > 0);
            double abs_s = std::abs(s);
            
            gpiod_line * active_line = forward ? line_fwd_ : line_bwd_;
            gpiod_line * inactive_line = forward ? line_bwd_ : line_fwd_;

            auto on_time = std::chrono::microseconds(static_cast<long>(period.count() * abs_s));
            auto off_time = period - on_time;

            // Upewnij si�, �e nieaktywny pin jest wy��czony
            gpiod_line_set_value(inactive_line, 0);

            // Faza ON
            if (on_time.count() > 0) {
                gpiod_line_set_value(active_line, 1);
                std::this_thread::sleep_for(on_time);
            }
            // Faza OFF
            if (off_time.count() > 0) {
                gpiod_line_set_value(active_line, 0);
                std::this_thread::sleep_for(off_time);
            }
        }
    }
};


class MotorController : public rclcpp::Node {
public:
    MotorController() : Node("motor_controller") {

        chip_ = gpiod_chip_open(GPIO_CHIP);
        if (!chip_) throw std::runtime_error("Nie mo�na otworzy� " + std::string(GPIO_CHIP));

        auto get_line = [&](int pin) -> gpiod_line * {
            gpiod_line * l = gpiod_chip_get_line(chip_, pin);
            if (!l) throw std::runtime_error("Nie mo�na uzyska� linii GPIO " + std::to_string(pin));
            if (gpiod_line_request_output(l, "motor_ctrl", 0) < 0)
                throw std::runtime_error("B��d request_output dla GPIO " + std::to_string(pin));
            return l;
        };

        line_lf_ = get_line(PIN_LEFT_FWD);
        line_lb_ = get_line(PIN_LEFT_BWD);
        line_rf_ = get_line(PIN_RIGHT_FWD);
        line_rb_ = get_line(PIN_RIGHT_BWD);

        motor_left_  = std::make_unique<SoftPwmMotor>(line_lf_, line_lb_);
        motor_right_ = std::make_unique<SoftPwmMotor>(line_rf_, line_rb_);

        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(cmd_mutex_);
                tgt_lin_ = msg->linear.x;
                tgt_ang_ = msg->angular.z;
            });

        running_ = true;
        rt_thread_ = std::thread(&MotorController::control_loop, this);

        sched_param sp{};
        sp.sched_priority = RT_PRIORITY;
        if (pthread_setschedparam(rt_thread_.native_handle(), SCHED_FIFO, &sp) != 0)
            RCLCPP_WARN(get_logger(), "Nie ustawiono SCHED_FIFO. Uruchomiono normalnie.");

        RCLCPP_INFO(get_logger(), 
            "MotorController (SoftPWM 4-pin) uruchomiony  chip=%s  loop=%.0f Hz", 
            GPIO_CHIP, CONTROL_LOOP_HZ);
    }

    ~MotorController() {
        running_ = false;
        if (rt_thread_.joinable()) rt_thread_.join();

        motor_left_->stop();
        motor_right_->stop();

        gpiod_line_release(line_lf_);
        gpiod_line_release(line_lb_);
        gpiod_line_release(line_rf_);
        gpiod_line_release(line_rb_);
        gpiod_chip_close(chip_);

        RCLCPP_INFO(get_logger(), "Zwolniono zasoby GPIO.");
    }

private:
    double tgt_lin_{0.0}, tgt_ang_{0.0};
    double cur_lin_{0.0}, cur_ang_{0.0};
    std::mutex cmd_mutex_;

    gpiod_chip * chip_{nullptr};
    gpiod_line * line_lf_{nullptr}, * line_lb_{nullptr};
    gpiod_line * line_rf_{nullptr}, * line_rb_{nullptr};
    std::unique_ptr<SoftPwmMotor> motor_left_, motor_right_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

    std::thread       rt_thread_;
    std::atomic<bool> running_{false};

    static double ramp(double cur, double tgt) {
        double diff = tgt - cur;
        if (std::abs(diff) <= ACCEL_STEP) return tgt;
        return cur + std::copysign(ACCEL_STEP, diff);
    }

    void control_loop() {
        using Clock    = std::chrono::steady_clock;
        using Duration = std::chrono::duration<double>;

        const auto period = Duration(1.0 / CONTROL_LOOP_HZ);
        auto next = Clock::now();

        while (running_) {
            double tgt_lin, tgt_ang;
            {
                std::lock_guard<std::mutex> lk(cmd_mutex_);
                tgt_lin = tgt_lin_;
                tgt_ang = tgt_ang_;
            }

            cur_lin_ = ramp(cur_lin_, tgt_lin);
            cur_ang_ = ramp(cur_ang_, tgt_ang);

            double left  = cur_lin_ + cur_ang_;
            double right = cur_lin_ - cur_ang_;

            double scale = std::max({std::abs(left), std::abs(right), 1.0});
            motor_left_ ->drive(left  / scale);
            motor_right_->drive(right / scale);

            next += std::chrono::duration_cast<Clock::duration>(period);
            std::this_thread::sleep_until(next);
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<MotorController>());
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("motor_controller"), "B��d: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}