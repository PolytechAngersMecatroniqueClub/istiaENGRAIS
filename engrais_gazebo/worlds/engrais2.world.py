
outputfile = "engrais2.world"
myworld = open(outputfile, "w")
# parameters:

nb_rows = 4
nb_plants_rows = 30
space_between_rows = 3
row_width = 0.1
row_lenght = 10
nb_outliers = 20

myworld.write('<?xml version="1.0" ?>\n')
myworld.write('<sdf version="1.4">\n')
myworld.write('  <world name="default">\n')
myworld.write('    <include>\n')
myworld.write('      <uri>model://ground_plane</uri>\n')
myworld.write('    </include>\n')
myworld.write('    <include>\n')
myworld.write('      <uri>model://sun</uri>\n')
myworld.write('    </include>\n')

for i in range(0, nb_rows):
    myworld.write('    <population name="row'+str(i)+'">\n')
    myworld.write('      <model name="plantgreen'+str(i)+'">\n')
    myworld.write('        <include>\n')
    myworld.write('          <uri>model://plantgreen</uri>\n')
    myworld.write('        </include>\n')
    myworld.write('      </model>\n')
    myworld.write('      <pose>'+str(row_lenght/2.0)+' '+str(space_between_rows*(i)-space_between_rows/2)+' 0 0 0 0</pose>\n')
    myworld.write('      <box>\n')
    myworld.write('        <size>' + str(row_lenght) + ' ' + str(row_width) + ' 0.01</size>\n')
    myworld.write('      </box>\n')
    myworld.write('      <model_count>'+str(nb_plants_rows)+'</model_count>\n')
    myworld.write('      <distribution>\n')
    myworld.write('        <type>random</type>\n')
    myworld.write('      </distribution>\n')
    myworld.write('    </population>\n')


myworld.write('  </world>\n')
myworld.write('</sdf>\n')


