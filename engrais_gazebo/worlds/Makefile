all: engrais engrais2 engrais3

engrais:
	rosrun xacro xacro -o engrais.world engrais.world.xacro
	sed -i 's/<sdf version="1.5" xmlns:xacro="http:\/\/www.ros.org\/wiki\/xacro">/<sdf version="1.5">/g' engrais.world

engrais2:
	python3 engrais2.world.py

engrais3:
	python3 engrais3.world.py


