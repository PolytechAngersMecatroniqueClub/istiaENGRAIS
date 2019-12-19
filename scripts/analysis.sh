#!/bin/bash
numIterations=5
baseDir="../../Results"
worldsDir="../engrais_gazebo/worlds"

simResDir="Simulation Results"
resultsDir="Analysis Results"
plantsDir="Plants Position"

resDirectory=("engrais" "engrais2" "engrais3" "engrais4")

algorithm=("Pearl" "RubyPure" "RubyGenetic" "RubyGeneticOnePoint" "RubyGeneticOnePointPosNeg" "RubyGeneticOnePointPosNegInfinite")


mkdir -p "$baseDir/$plantsDir"

printf "Getting Plants Position:\n\n"

cd "${worldsDir}/"

for environment in ${resDirectory[@]}
do
	./plantsLocation.py "${environment}.world" "../${baseDir}/${plantsDir}/${environment}.world.csv"
done

cd "../../scripts"

printf "Processing Trajectories : \n\n"

for environment in ${resDirectory[@]}
do
	printf "\t$environment : \n"
	for algo in ${algorithm[@]}
	do
		printf "\t\t$algo "

		mkdir -p "$baseDir/$resultsDir/$environment/$algo"

		python3 .resultAnalysis.py "$algo" "$environment" "$baseDir/$simResDir" "$baseDir/$resultsDir/$environment/$algo" "$baseDir/$plantsDir" "$numIterations"

		printf "OK\n"
	done

	printf "\n"

done

printf "Finished\n\n"
