mkdir -p ~/temp

yes | sudo apt install git

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
yes | sudo apt update
yes | sudo apt install ros-melodic-desktop-full

apt search ros-melodic

yes | sudo rosdep init
rosdep update

yes | sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
yes | sudo apt-get install unzip
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

yes | sudo apt-get update
yes | sudo apt-get install gazebo9
yes | sudo apt-get install libgazebo9-dev
yes | sudo apt upgrade libignition-math2

yes | sudo apt install ros-melodic-gazebo-ros
yes | sudo apt install ros-melodic-controller-manager
yes | sudo apt install ros-melodic-velocity-controllers
yes | sudo apt install ros-melodic-controller-interface
yes | sudo apt install ros-melodic-joint-state-controller
yes | sudo apt install ros-melodic-effort-controllers

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

echo "Done"
