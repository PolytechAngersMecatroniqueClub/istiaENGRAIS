mkdir -p ~/temp

sudo apt install git

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full

apt search ros-melodic

sudo rosdep init
rosdep update

sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install unzip
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install gazebo9
sudo apt-get install libgazebo9-dev
sudo apt upgrade libignition-math2

sudo apt install ros-melodic-gazebo-ros
sudo apt install ros-melodic-controller-manager
sudo apt install ros-melodic-velocity-controllers
sudo apt install ros-melodic-controller-interface
sudo apt install ros-melodic-joint-state-controller
sudo apt install ros-melodic-effort-controllers

cd ~/temp/
git clone https://github.com/PolytechAngersMecatroniqueClub/istiaENGRAIS.git

mkdir -p ~/catkin_ws/src

rm -rf ~/catkin_ws/src/istiaENGRAIS
cp -ar istiaENGRAIS ~/catkin_ws/src/
rm -rf istiaENGRAIS

cd ~/catkin_ws/src/istiaENGRAIS/engrais_control/src/.fuzzylite-6.0

unzip fuzzylite-6.0.zip -d ~/temp
cd ~/temp/fuzzylite-6.0/fuzzylite/

chmod +x ./build.sh

./build.sh all

cp -ar release/bin/ ~/catkin_ws/src/istiaENGRAIS/engrais_control/src/.fuzzylite-6.0/
cd ~/catkin_ws/src/istiaENGRAIS/
git checkout raspberry_pi
cd ~/temp/fuzzylite-6.0/fuzzylite/

cp -ar release/bin/ ~/catkin_ws/src/istiaENGRAIS/engrais_control/src/.fuzzylite-6.0/
cd ~/catkin_ws/src/istiaENGRAIS/
git checkout master

cd ~/catkin_ws/

source /opt/ros/melodic/setup.bash

catkin_make

printf "\nsource /opt/ros/melodic/setup.bash" >> ~/.bashrc

printf "\nsource ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

rm -rf ~/temp

source /opt/ros/melodic/setup.bash
