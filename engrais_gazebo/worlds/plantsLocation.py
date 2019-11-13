#!/usr/bin/python

"""python script to extract the coordinates of elements in a world file
to use the script :
    python 3convert_world_csv [file_name.world]
It will save the coordinates in a csv file named file_name.world.csv
"""
import sys
import re

def open_file(file_name, mode):
    try:
        file = open(file_name, mode)
    except IOError:
        print("[ERROR] can not open the file \'" + file_name + '\'')
        sys.exit(2)

    return file


def main():

    # try to open the file to process
    print "[INFO] Processing the file: \'" + sys.argv[1] + '\''

    input_file = open_file(sys.argv[1], "r")
    output_file = open_file(sys.argv[2], "w")

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

    print "[INFO] Coordinates saved into the file: \'" + sys.argv[2] + '\'\n'


if __name__ == "__main__":
    if len(sys.argv) != 3:  # given file name
        print("[ERROR] Wrong argument, expecting 'input_file', 'output_file'")
        sys.exit(1)

    main()