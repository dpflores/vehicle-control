import matplotlib.pyplot as plt
import os

POSITION_OUTPUT_FILE = "controller_output/trajectory.txt"

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)



graph_data = open(POSITION_OUTPUT_FILE,'r').read()
lines = graph_data.split('\n')

xs = []
ys = []

error_s = []
for line in lines:
    if len(line) > 1:
        x, y, v, t= line.split(',')
        
        xs.append(float(x))
        ys.append(float(y))
    



ax1.plot(xs, ys, 'b-')
ax1.set_xlabel("x position (m)")
ax1.set_ylabel("y position (m)")



plt.show()