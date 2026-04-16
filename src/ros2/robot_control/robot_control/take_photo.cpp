#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include <iostream>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("take_photo_trigger");
    auto pub = node->create_publisher<std_msgs::msg::Empty>("/camera/take_snapshot", 10);

    RCLCPP_INFO(node->get_logger(), "=== PILOT APARATU GOTOWY ===");
    RCLCPP_INFO(node->get_logger(), "Wciœnij [ENTER], aby zrobiæ zdjêcie.");
    RCLCPP_INFO(node->get_logger(), "Wpisz 'q' i [ENTER], aby zamkn¹æ program.");

    while (rclcpp::ok()) {
        char c = std::cin.get();
        if (c == 'q') {
            break;
        }
        if (c == '\n') {
            pub->publish(std_msgs::msg::Empty());
            RCLCPP_INFO(node->get_logger(), "--> Wys³ano komendê zrobienia zdjêcia!");
        }
    }

    rclcpp::shutdown();
    return 0;
}