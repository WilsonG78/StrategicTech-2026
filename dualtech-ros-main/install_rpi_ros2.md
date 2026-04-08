## Instalacja ROS 2 Jazzy na Raspberry Pi 5 (Debian 12)

1. **Aktualizacja systemu:**
	```bash
	sudo apt update
	sudo apt upgrade -y
	sudo reboot
	```

2. **Dodanie repozytoriów ROS 2:**
	```bash
	sudo apt install -y software-properties-common
	sudo add-apt-repository universe
	sudo apt update
	sudo apt install -y curl gnupg2 lsb-release
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	sudo apt update
	```


3. **Instalacja zależności systemowych (przed budowaniem/instalacją ROS 2):**
	```bash
	chmod +x install_rpi_ros2_deps.sh
	sudo ./install_rpi_ros2_deps.sh
	```

4. **Inicjalizacja rosdep:**
	```bash
	sudo rosdep init
	rosdep update
	```
    
	Jeżeli wystąpił błąd z uprawnieniami podczas instalacji lub inicjalizacji rosdep:
	1. Utwórz katalog:
		```bash
		sudo mkdir -p /etc/ros/rosdep/sources.list.d
		```
	2. Zainicjuj rosdep:
		```bash
		sudo rosdep init
		```
	3. Zaktualizuj rosdep:
		```bash
		rosdep update
		```


5. **Pobranie i budowa ROS 2 Jazzy ze źródeł:**

	> **Uwaga:** Instalacja i budowa może zająć do 2 godzin, jeśli nie wystąpią żadne problemy.
	>
	> Podczas budowania mogą pojawić się problemy z brakiem bibliotek (nawet po instalacji zależności przez install_rpi_ros2_deps.sh) lub z kompatybilnością. Warto wtedy usunąć foldery `build/`, `install/` oraz `log/` i spróbować ponownie uruchomić kompilację.
	```bash
	mkdir -p ~/ros2_jazzy/src
	cd ~/ros2_jazzy
	wget https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos
	vcs import src < ros2.repos
	rosdep install --from-paths src --ignore-src -r -y
	colcon build --symlink-install --parallel-workers 6
	```

6. **Dodanie ROS do ścieżki:**
	```bash
	echo "source ~/ros2_jazzy/install/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	```

7. **Test:**
	```bash
	ros2 run demo_nodes_cpp talker
	```