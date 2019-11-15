mkdir -p ~/catkin_ws/src/

cd ../../

cp -r ./istiaENGRAIS

printf "ROS Installation... "

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-melodic-desktop-full

sudo rosdep init

rosdep update

sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

printf "OK\n\n"

