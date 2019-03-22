all: green red clean

green:
	@echo "--> Generating the plantgreen.sdf file from the xacro"
	rosrun xacro xacro -o plantgreen/plantgreen.sdf plantgreen/plantgreen.xacro

red:
	@echo "--> Generating the plantred.sdf file from the xacro"
	rosrun xacro xacro -o plantred/plantred.sdf plantred/plantred.xacro

clean:
	@echo "--> Cleaning the file by removing the xmlns:xacro tags"
	sed -i 's/<sdf version="1.5" xmlns:xacro="http:\/\/www.ros.org\/wiki\/xacro">/<sdf version="1.5">/g' plantgreen/plantgreen.sdf
	sed -i 's/<sdf version="1.5" xmlns:xacro="http:\/\/www.ros.org\/wiki\/xacro">/<sdf version="1.5">/g' plantred/plantred.sdf


