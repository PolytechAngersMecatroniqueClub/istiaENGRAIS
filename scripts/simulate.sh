#!/bin/bash

simResPath="/home/usrlocal/Simulation Results" 

resDirectory=("engrais" "engrais2" "engrais3" "engrais4")

algorith=("Pearl" "RubyPure" "RubyGenetic" "RubyGeneticOnePoint" "RubyGeneticOnePointPosNeg" "RubyGeneticOnePointPosNegInfinite")

printf "\nLaunching Simulation:\n\n"

for environment in ${resDirectory[@]}
do
	printf "$environment Simulation Begin:"

	for algo in ${algorith[@]}
	do
		mkdir -p "$simResPath/$environment/$algo"

		printf "\n\t$algo : "

		for i in 0 1 2 3 4
		do
			tmux new -d -s gazebo #Create gazebo session
			tmux send -t gazebo.0 "roslaunch engrais_gazebo engrais_world.launch world:=$environment" C-m #Launch simulation environment

			sleep 1m #Wait gazebo to complete


			tmux new -d -s robot_control #Create robot_control session
			tmux send-keys -t robot_control.0 "roslaunch engrais_control simulation.launch algorithm:=$algo file_name:='$simResPath/$environment/$algo/${algo}_results_${i}.csv' execution_file_name:='$simResPath/$environment/$algo/${algo}_execution_${i}.csv' >> /home/usrlocal/catkin_ws/Output/${algo}_${environment}_${i}" C-m #Launch simulation environment

			sleep 45m


			tmux kill-session -t gazebo
			tmux kill-session -t robot_control
			sleep 2m

			printf "|"
			if [ $i -eq 4 ]
			then
				printf "  "
			fi
		done
	done

	printf "\n\n"
done
echo "Simulation finished"