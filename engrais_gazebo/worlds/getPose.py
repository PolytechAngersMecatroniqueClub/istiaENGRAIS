#!/usr/bin/python

"""python script to extract the coordinates of elements in a world file
to use the script :
    python 3convert_world_csv [file_name.world]
It will save the coordinates in a csv file named file_name.world.csv
"""

import sys
import re

default_file = "engrais3.world"
file_name = ""

# test if the name of the file has been given as an argument
if len(sys.argv) == 1:  # no argument
    file_name = default_file
elif len(sys.argv) == 2:  # given file name
    file_name = sys.argv[1]
else:
    print("[ERROR] convert_world [file_name.world]")
    sys.exit(1)

# try to open the file to process
print("[INFO] Processing the file:", file_name)
try:
    input_file = open(file_name, 'r')
except IOError:
    print("[ERROR] can not open the file", file_name)
    sys.exit(2)

# try to open the file to save the coordinates
try:
    output_name = "/home/usrlocal/Plants Position/" + file_name + ".csv"
    output_file = open(output_name, 'w')  # if the file exists it is erased
except IOError:
    print("[ERROR] can not open the file", output_name)
    sys.exit(3)

# write the first line in the output file
output_file.write("type;x;y\n")

nextPlant = "Green"

# we loop over all the lines of the file
for i, line in enumerate(input_file):
    # we select the lines with a <pose> markup
    if re.match(r"(.)*model(.)*name(.)*", line):
        redPlant = re.findall("plantred_[0-9]+[0-9]*",line)

        if redPlant:
            nextPlant = "Red"

        else:
            nextPlant = "Green"

    elif re.match(r"(.)*pose(.)*", line):
        # we select all the float numbers in the line
        test = re.findall("\-*[0-9]+\.*[0-9]*",line)

        output_file.write(nextPlant)  # the first float corresponds to the x coordinate
        output_file.write(";")
        output_file.write(test[0])  # the first float corresponds to the x coordinate
        output_file.write(";")
        output_file.write(test[1])  # the second float corresponds to the x coordinate
        output_file.write("\n")

input_file.close()
output_file.close()

print("[INFO] Coordinates saved into the file:", output_name)
