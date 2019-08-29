export ROS_IP=`hostname -I`
. ~/catkin_ws/devel/setup.sh
exec "$@"