#sudo apt-get install sshpass

all_IPs=("192.168.10.101" "192.168.10.102" "192.168.10.103" "192.168.10.104" "192.168.10.105" "192.168.10.110")

all_usernames=("engrais" "engrais" "engrais" "engrais" "jetson-tx2" "usrlocal")

all_pass=("engrais" "engrais" "engrais" "engrais" "jetsontx2" "usrlocal2019")

for ((i=0;i<${#all_IPs[@]};i++)); 
do 
	sshpass -p ${all_pass[$i]} ssh -oHostKeyAlgorithms='ssh-rsa' -oStrictHostKeyChecking=no ${all_usernames[$i]}@${all_IPs[$i]} 'printf "\n\nsource /opt/ros/melodic/setup.bash\nsource ~/catkin_ws/devel/setup.bash\n\nIP=\`hostname -I\`\n\nIP=\`echo \"\$IP\" | cut -d'"' '" ' -f 1\`\n\nexport ROS_MASTER_IP=\$IP\nexport ROS_MASTER_URI=http://\$ROS_MASTER_IP:11311\nexport ROS_IP=\$IP\n\nif [[ \$IP == \"192.168.10.\"* ]];\nthen\n\techo \"WARN: Connected to DLINK, Ros Master IP: \$ROS_MASTER_IP, this IP: \$IP\"\nelse\n\techo \"Warn: Ros Master IP: \$ROS_MASTER_IP, this IP: \$IP\"\nfi\n\n" >> ~/.bashrc && exit'
done