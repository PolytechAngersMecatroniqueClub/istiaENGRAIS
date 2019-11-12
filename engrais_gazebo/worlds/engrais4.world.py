#!/usr/bin/env python
#pyhton code that generates a world with random plants and weeds
import random as random

def is_xOK(x, row): # to add some holes in the rows
    ret_val = False
    if  row == 0:
        ret_val = True
    elif row == 1:
        if 0 <= x <= 5 or 15 <= x <= 20:
            ret_val = True
    elif row == 2:
        if 0 <= x <= 10 or 20 <= x <= 25:
            ret_val = True
    elif row == 3:
        if 0 <= x <= 5 or 10 <= x <= 15:
            ret_val = True
    elif row == 4:
        if 10 <= x <= 20:
            ret_val = True
    elif row == 5:
        ret_val = True
    return ret_val

def is_yOK(y, nb_rows, space_between_rows, row_width): # to add some holes in the rows
    flag = True
    delta = 0.1

    for i in range(0, nb_rows):
        if (space_between_rows*i-space_between_rows/2.0-row_width/2.0 - delta) <= y <= (space_between_rows*i-space_between_rows/2.0+row_width/2.0 + delta):
            flag = False

    return flag

outputfile = "engrais4.world"  # name of the generated world
myworld = open(outputfile, "w")

# parameters:
nb_rows = 6             # nb rows
nb_plants_rows = 100     # nb plants in a row
space_between_rows = 1  # space between two rows (from center to center)
row_width = 0.1        # width of a row
row_lenght = 25         # length of a row
nb_outliers = 100        # nb of outliers in the all field
space_min_plants = 0.15 # minimal space between two generated plants

myworld.write('<?xml version="1.0" ?>\n')
myworld.write('<sdf version="1.4">\n')
myworld.write('  <world name="default">\n')
myworld.write('    <include>\n')
myworld.write('      <uri>model://ground_plane</uri>\n')
myworld.write('    </include>\n')
myworld.write('    <include>\n')
myworld.write('      <uri>model://sun</uri>\n')
myworld.write('    </include>\n')

plants = []

print("Generation of green plants...")
for i in range(0, nb_rows):
    row_array = []
    # row_array.resize(nb_plants_rows)
    nb_append = 0
    while(nb_append != nb_plants_rows):
        x = random.uniform(0, row_lenght)
        y = random.uniform(space_between_rows*i-space_between_rows/2.0-row_width/2.0, space_between_rows*i-space_between_rows/2.0+row_width/2.0)
        flag = True
        for plant in row_array:
            if pow(x-plant[0],2) + pow(y-plant[1],2) < pow(space_min_plants,2):
                flag = False
        if flag:
            if is_xOK(x, i):
                row_array.append([x, y])
                plants.append([x, y])

                myworld.write('      <model name="plantgreen_' + str(i) + '_' + str(nb_append) + '">\n')
                myworld.write('        <include>\n')
                myworld.write('          <uri>model://plantgreen</uri>\n')
                myworld.write('        </include>\n')
                myworld.write('        <pose>'+str(x)+' '+str(y)+' 0 0 0 0</pose>\n')
                myworld.write('      </model>\n')
            nb_append += 1
print("... Generation of green plants OK")

print("Generation of red plants...")
nb_append = 0
while(nb_append != nb_outliers):
    x = random.uniform(0, row_lenght)
    y = random.uniform(-space_between_rows/2.0+row_width/2.0+space_min_plants, space_between_rows*nb_rows-space_between_rows/2.0-row_width/2.0-space_min_plants)
    flag = True
    for plant in plants:
        if pow(x-plant[0],2) + pow(y-plant[1],2) < pow(space_min_plants,2):
            flag = False
    if flag and is_yOK(y, nb_rows, space_between_rows, row_width):
        nb_append += 1

        myworld.write('      <model name="plantred_' + str(nb_append) + '">\n')
        myworld.write('        <include>\n')
        myworld.write('          <uri>model://plantred</uri>\n')
        myworld.write('        </include>\n')
        myworld.write('        <pose>'+str(x)+' '+str(y)+' 0 0 0 0</pose>\n')
        myworld.write('      </model>\n')
        plants.append([x, y])

print("... Generation of red plants OK")

myworld.write('  </world>\n')
myworld.write('</sdf>\n')