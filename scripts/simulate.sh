#!/bin/bash

baseDir="../../"

simResDir="Results/Simulation Results"
outDir="Results/Output" 

resDirectory=("engrais" "engrais2" "engrais3" "engrais4")

algorithm=("Pearl" "RubyPure" "RubyGenetic" "RubyGeneticOnePoint" "RubyGeneticOnePointPosNeg" "RubyGeneticOnePointPosNegInfinite")

printf "\nLaunching Simulation:\n\n"

for environment in ${resDirectory[@]}
do
	printf "$environment Simulation Begin:"

	for algo in ${algorithm[@]}
	do

		mkdir -p "$baseDir/$simResDir/$environment/$algo"
		mkdir -p "$baseDir/$outDir/$environment/$algo"

		printf "\n\t$algo : "

		for i in 0 1 2 3 4
		do
			tmux new -d -s gazebo #Create gazebo session
			tmux send -t gazebo.0 "roslaunch engrais_gazebo engrais_world.launch world:=$environment" C-m #Launch simulation environment

			sleep 1m #Wait gazebo to complete


			tmux new -d -s robot_control #Create robot_control session
			tmux send-keys -t robot_control.0 "roslaunch engrais_control simulation.launch algorithm:='$algo' file_name:='../catkin_ws/src/$simResDir/$environment/$algo/${algo}_results_${i}.csv' execution_file_name:='../catkin_ws/src/$simResDir/$environment/$algo/${algo}_execution_${i}.csv' >> $baseDir/${outDir}/${environment}/${algo}/${algo}_${environment}_${i}" C-m #Launch simulation environment

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

#algo=${algorith[0]}
#environment=${resDirectory[0]}
#i=0

#echo "roslaunch engrais_control simulation.launch algorithm:='${algo}' file_name:='$simResPath/$environment/$algo/${algo}_results_${i}.csv' execution_file_name:='$simResPath/$environment/$algo/${algo}_execution_${i}.csv' >> './src/${outPath}/${environment}/${algo}/${algo}_${environment}_${i}'"







