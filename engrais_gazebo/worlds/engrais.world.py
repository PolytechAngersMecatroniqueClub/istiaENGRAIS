#!/usr/bin/env python
#pyhton code that generates a world with random plants and weeds
import random as random

outputfile = "engrais.world"  # name of the generated world
myworld = open(outputfile, "w")

# world parameters:
nb_rows = 6            # number of rows
nb_plants_rows = 40    # number of plants in a row
space_between_rows = 1.5 # space between the rows
row_width = 0.0        # width of a row
row_lenght = 10       # length of a row
space_plants = 0.3 # minimal space between two generated plants

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
    for j in range(0, nb_plants_rows):
        #print (nb_append, nb_plants_rows, i
        x = j * space_plants
        y = i * space_between_rows - space_between_rows/2.0
        
        myworld.write('      <model name="plantgreen_' + str(i) + '_' + str(j) + '">\n')
        myworld.write('        <include>\n')
        myworld.write('          <uri>model://plantgreen</uri>\n')
        myworld.write('        </include>\n')
        myworld.write('        <pose>'+str(x)+' '+str(y)+' 0 0 0 0</pose>\n')
        myworld.write('      </model>\n')

print("... Generation of green plants OK")


myworld.write('  </world>\n')
myworld.write('</sdf>\n')




