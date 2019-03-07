# ENGRAIS
This repository contains all the packages to simulate and control a robot through an unknown field, using LIDAR sensors.

# Organization
This project contains 3 parts, where 2 of them are related to simulation, and 1 for control.

- Robot model and environment: robot_model is the folder containing all the robot's parts, physics as well as obstacles (in cylinder form).
- Robot plugin: robot_plugin is the folder containing the files to establish a communication betwen ROS and Gazebo. Without it, it's impossible to control robot's wheels or get sensor's data outside gazebo.
- Robot control: robot_control is the folder that contains the ROS nodes that get the sensor's data, calculate lines, send the models to rviz and (eventualy) send control commands to the robot's wheels.

# How to use ?
In order to use all the modules, some settings must be done.

## Model
To use the model, put the "project_robot_sphere" folder inside "~/.gazebo/models", no additional settings required.

## Plugin
To use the plugin, place the "robot_plugin" folder at any place. Then, every time you wish to use the plubin, you must open a terminal in "~/<folder_path>/robot_plugin/build", run the command "make" (and run the command again if you change any file inside "~/<folder_path>/robot_plugin" and run the command "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/<folder_path>robot_plugin/build". My recommendation is to put the last command inside ".bashrc" in the home folder. Finally, use "gazebo ../robot.world" command to run gazebo. To summarize: 
1) Open a terminal inside HOME (control++shift+t) and run "gedit .bashrc".
2) Paste the command "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/<folder_path>/robot_plugin/build" as the last line in the file, then save and close the file reopen the terminal.
3) Navigate to the plugin "cd ~/<folder_path>/robot_plugin/build".
4) Use "make" command.
5) Use "gazebo --verbose ../robot.world" to run gazebo with the plugin.
6) To test if everything is fine, run "rosnode list" and check if "/gazebo_node" is on the list. Then run "rostopic list" and check if "/robot/sensor/data", "/robot/sensor/vel_cmd", "/robot/wheels/instant_vel_cmd" and "/robot/wheels/vel_cmd" are there.

## Control
To use the robot control, I recommend to run "gedit .bashrc" in home folder, and add the line "source ~/<catkin_path>/devel/setup.bash" as the last line of the file. Then, save and close the file and reopen terminal. Now that your catkin workspace is the ROS default, navigate to "cd ~/<catkin_path>/src" and paste the "robot_control" folder there. Then, open a terminal in "~/<catkin_path>" and run "catkin_make". To summarize:

1) Open "gedit .bashrc" in home folder.
2) Paste "source ~/<catkin_path>/devel/setup.bash" as the last line in the file, save and close it then reopen the terminal.
3) Paste the "robot_control" folder inside "~/<catkin_path>/src".
4) Open a terminal in "~/<catkin_path>" and run "catkin_make".

Now, inside this folder you'll find 2 ROS nodes, "robot_control" and "robot_keyboard_control", the first used to get the sensor's data comming from gazebo, and calculate lines, and the latter used to move the robot inside gazebo using the keyboard's arrows (warning: moving the robot is a bit awkward). To use them, run:

- Robot Control: "roscore" in a terminal, "rosrun robot_control robot_control" in another, and visualize everything in "rviz". Don't forget to have gazebo running in another terminal.
- Robot Keyboard Control: "roscore" in a terminal, "rosrun robot_control robot_keyboard_control" in another. Don't forget to have gazebo running in another terminal.


# Common changes
Some common changes to same of these files are:

- In "~/.gazebo/models/project_robot_sphere/model.sdf", you can change the minimum or maximum angle searching for "min_angle" and "max_angle", and the number of sensor rays searching for "samples"