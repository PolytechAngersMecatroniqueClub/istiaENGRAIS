#!/bin/bash

simResPath="/home/usrlocal/Simulation Results" 

resDirectory=("engrais1" "engrais2" "engrais3")

algorith=("Pearl" "RubyPure" "RubyGenetic" "RubyGeneticOnePoint" "RubyGeneticOnePointPosNeg" "RubyGeneticOnePointPosNegInfinite")


printf "\nLaunching Simulation:"
tmux new -d -s kill_model #Create kill_model session

for environment in ${resDirectory[1]}
do

	tmux new -d -s gazebo #Create gazebo session
	tmux send -t gazebo.0 "roslaunch engrais_gazebo engrais_world.launch world:=$environment" C-m #Launch simulation environment

	printf "\n\n$environment launched, waiting for initialization\n"
	sleep 10 #Wait gazebo to complete
	printf "Initialization Complete\n\n"

	tmux send -t kill_model.0 'rosservice call gazebo/delete_model "{model_name: engrais_robot}"' C-m
	sleep 1 #Wait service to finish

	printf "$environment Simulation Begin:"

	for algo in ${algorith[5]}
	do
		mkdir -p "$simResPath/$environment/$algo"

		printf "\n\t$algo : "

		for i in 0
		do
			tmux new -d -s robot_control #Create robot_control session
			tmux send-keys -t robot_control.0 "roslaunch engrais_control simulation.launch algorithm:=$algo file_name:='$simResPath/$environment/$algo/${algo}_results_${i}.csv'" C-m #Launch simulation environment

			sleep 3m

			tmux kill-session -t robot_control
			tmux send -t kill_model.0 'rosservice call gazebo/delete_model "{model_name: engrais_robot}"' C-m

			sleep 5

			printf "|"
			if [ $i -eq 4 ]
			then
				printf "  "
			fi
		done
	done

	printf "\n\n"

	tmux kill-session -t gazebo

	sleep 1m
done

printf "\n\n"
tmux kill-session -t kill_model

echo "Simulation finished"