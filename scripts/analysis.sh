#!/bin/bash
numIterations=1
baseDir="../../Results"
worldsDir="../engrais_gazebo/worlds"

simResDir="Simulation Results"
resultsDir="Analysis Results"
plantsDir="Plants Position"

resDirectory=("engrais" "engrais2" "engrais3" "engrais4")

algorithm=("Pearl" "RubyPure" "RubyGenetic" "RubyGeneticOnePoint" "RubyGeneticOnePointPosNeg" "RubyGeneticOnePointPosNegInfinite")


mkdir -p "$baseDir/$plantsDir"
mkdir -p "$baseDir/$resultsDir"

printf "Getting Plants Position:\n\n"

cd "${worldsDir}/"

for environment in ${resDirectory[@]:2}
do
	./plantsLocation.py "${environment}.world" "../${baseDir}/${plantsDir}/${environment}.world.csv"
done

cd "../../scripts"


for environment in ${resDirectory[3]}
do
	for algo in ${algorithm[0]}
	do
		./resultAnalysis.py "$algo" "$environment" "$baseDir/$simResDir" "$baseDir/$resultsDir" "$baseDir/$plantsDir" "$numIterations"
	done
done
