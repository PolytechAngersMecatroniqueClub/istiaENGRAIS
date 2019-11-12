#!/usr/bin/python

import os, sys
import re
import plotly.graph_objects as go
import numpy as np

def open_file(file_name):
	try:
		file = open(file_name, "r+")
	except IOError:
		print("[ERROR] can not open the file \'" + file_name + '\'')
		sys.exit(2)

	return file


def main():
	algorithm = sys.argv[1]
	environment = sys.argv[2]

	input_directory = str(os.path.expanduser('~')) + "/" + sys.argv[3]
	output_directory = str(os.path.expanduser('~')) + "/" + sys.argv[4]
	plants_directory = str(os.path.expanduser('~')) + "/" + sys.argv[5]

	if not os.path.exists(output_directory):
	    os.makedirs(output_directory)

	plants_file = open_file(plants_directory + "/" + environment + ".world.csv")

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

	fig.update_xaxes(range=[-2.0, 30.0])
	fig.update_yaxes(range=[-1.0, 5.5])
	 
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

	for i in range(1):
		t = []
		x = []
		y = []
		z = []
		r = []
		p = []
		ya = []

		input_file = open_file(input_directory + "/" + environment + "/" + algorithm + "/" + algorithm + "_results_" + str(i) + ".csv") # ~/<path>/engrais4/Pearl/Pearl_results_0.csv

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

		fig.add_trace(go.Scatter(x=x, y=y, line_shape='linear', name = "trajectory_" + str(i)))

	fig.write_image(output_directory + "/" + algorithm + "_" + environment + ".svg")

	fig.show()


if __name__ == "__main__":
	if len(sys.argv) != 6:  # given file name
	    print("[ERROR] Wrong argument, expecting 'algorithm', 'environment', 'reults_directory', 'output_directory', 'plants_CSV_directory'")
	    sys.exit(1)

	main()