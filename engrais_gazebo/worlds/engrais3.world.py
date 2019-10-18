#!/usr/bin/env python
#pyhton code that generates a world with random plants and weeds
import random as random

outputfile = "engrais3.world"  # name of the generated world
myworld = open(outputfile, "w")

# parameters:
nb_rows = 4             # nb rows
nb_plants_rows = 20     # nb plants in a row
space_between_rows = 1  # space between two rows (from center to center)
row_width = 0.1        # width of a row
row_lenght = 5         # length of a row
nb_outliers = 20        # nb of outliers in the all field
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
            row_array.append([x, y])
            plants.append([x, y])
            nb_append += 1

            myworld.write('      <model name="plantgreen_' + str(i) + '_' + str(nb_append) + '">\n')
            myworld.write('        <include>\n')
            myworld.write('          <uri>model://plantgreen</uri>\n')
            myworld.write('        </include>\n')
            myworld.write('        <pose>'+str(x)+' '+str(y)+' 0 0 0 0</pose>\n')
            myworld.write('      </model>\n')
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
    if flag:
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




