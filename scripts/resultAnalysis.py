#!/usr/bin/python

import os, sys
import re
import plotly.graph_objects as go
import numpy as np

def open_file(file_name, mode):
	try:
		file = open(file_name, mode)
	except IOError:
		print("[ERROR] can not open the file \'" + file_name + '\'')
		sys.exit(2)

	return file


def main():
	algorithm = sys.argv[1]
	environment = sys.argv[2]

	input_directory = sys.argv[3]
	output_directory = sys.argv[4]
	plants_directory = sys.argv[5]

	nSim = int(sys.argv[6])

	if not os.path.exists(output_directory):
	    os.makedirs(output_directory)

	plants_file = open_file(plants_directory + "/" + environment + ".world.csv", "r")

	fig = go.Figure()

	fig.update_layout(
	    title="Robot Trajectory for: <br>" + algorithm + " on " + environment,
	    xaxis_title="X-coorditate (m)",
	    yaxis_title="Y-coorditate (m)",
	    font=dict(
	        family="Courier New, monospace",
	        size=18,
	        color="#7f7f7f"
	    )
	)

	maxX = 10.0
	maxY = 10.0

	movementRange = [0, 0]

	if(environment == "engrais"):
		fig.update_xaxes(range=[-2.0, 14.0])
		fig.update_yaxes(range=[-1.0, 7.0])
		maxX = 13.0
		maxY = 5.5

		rowsDistance = 1.5

		movementRange = [1, 11]

	elif(environment == "engrais2"):
		fig.update_xaxes(range=[-2.0, 12.0])
		fig.update_yaxes(range=[-1.0, 5.0])
		maxX = 11.0
		maxY = 3.5

		rowsDistance = 1.0

		movementRange = [1, 9]

	elif(environment == "engrais3"):
		fig.update_xaxes(range=[-2.0, 13.0])
		fig.update_yaxes(range=[-1.0, 7.5])
		maxX = 10.0
		maxY = 5.0

		rowsDistance = 1.3

		movementRange = [1, 8]

	elif(environment == "engrais4"):
		fig.update_xaxes(range=[-2.0, 30.0])
		fig.update_yaxes(range=[-1.0, 5.5])
		maxX = 23.0
		maxY = 3.5

		rowsDistance = 1.0

		movementRange = [1, 23]
	 
	red_plant_pos_x = []
	red_plant_pos_y = []

	green_plant_pos_x = []
	green_plant_pos_y = []

	for i, line in enumerate(plants_file):
		if(i == 0):
			continue

		data = line.split(';')

		if(data[0] == "Green"):
			green_plant_pos_x.append(data[1])
			green_plant_pos_y.append(data[2])

		elif(data[0] == "Red"):
			red_plant_pos_x.append(data[1])
			red_plant_pos_y.append(data[2])

	
	fig.add_trace(go.Scatter(x=red_plant_pos_x, y=red_plant_pos_y, marker_color='rgba(200, 0, 0, .8)', name="Weed"))
	fig.add_trace(go.Scatter(x=green_plant_pos_x, y=green_plant_pos_y, marker_color='rgba(0, 200, 0, .8)', name ="Plants"))
	fig.update_traces(mode='markers', marker_line_width=1.5, marker_size=4)

	resume_file = open_file(output_directory + "/" + algorithm + "_" + environment + "_results.csv", "w") # ~/<path>/engrais4/Pearl/Pearl_results_0.csv
	resume_file.write("iteration;Mean Err^2;Mean Points;Mean Exec Time(ms)\n")


	for i in range(nSim):
		t = []
		x = []
		y = []
		z = []
		r = []
		p = []
		ya = []

		err = []

		input_file = open_file(input_directory + "/" + environment + "/" + algorithm + "/" + algorithm + "_results_" + str(i) + ".csv", "r") # ~/<path>/engrais4/Pearl/Pearl_results_0.csv
		execution_file = open_file(input_directory + "/" + environment + "/" + algorithm + "/" + algorithm + "_execution_" + str(i) + ".csv", "r") # ~/<path>/engrais4/Pearl/Pearl_results_0.csv

		for j, line in enumerate(input_file):
			if(j == 0):
				continue

			data = line.split(';')

			t.append(float(data[0]))

			x.append(float(data[2]))
			y.append(float(data[3]))
			z.append(float(data[4]))

			r.append(float(data[6]))
			p.append(float(data[7]))
			ya.append(float(data[8]))


			if(movementRange[0] <= float(data[2]) <= movementRange[1]):
				for n in range(5):
					if(n * rowsDistance <= float(data[3]) + rowsDistance/2.0 <= rowsDistance * (n + 1)):
						err.append((float(data[3]) - n * rowsDistance) ** 2.0)


			if(x[len(x) - 1] >= maxX and y[len(y) - 1] >= maxY):
				break

		fig.add_trace(go.Scatter(x=x, y=y, line_shape='linear', name = "trajectory_" + str(i)))

		cont = 0
		nPointsSum = 0
		execTimeSum = 0

		for j, line in enumerate(execution_file):
			if(j == 0):
				continue

			data = line.split(';')

			if(j > 500):
				break

			nPointsSum += int(data[0])
			execTimeSum += float(data[1])
			cont += 1

		resume_file.write(str(i) + ";" + str(sum(err) / len(err)) + ";" + str(nPointsSum/float(cont)) + ";" + str(execTimeSum/float(cont)) + "\n")

	fig.write_image(output_directory + "/" + algorithm + "_" + environment + ".svg")

	#fig.show()

if __name__ == "__main__":
	if len(sys.argv) != 7:  # given file name
	    print("[ERROR] Wrong argument, expecting 'algorithm', 'environment', 'results_directory', 'output_directory', 'plants_CSV_directory', 'number of simulations'")
	    sys.exit(1)

	main()
