from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
import matplotlib.axes
import math
import time

data = numpy.loadtxt('path.txt')
fig = plt.figure()
ax = plt.axes()

plt.gca().set_aspect('equal', adjustable='box')
plt.xlim(-3.5, 3.5)
plt.ylim(-3.5, 3.5)

y = numpy.linspace(-3, 3, 1000)
matplotlib.axes.Axes.vlines(ax, -0.25/3, -3, 3, colors='k', linestyles='solid')
matplotlib.axes.Axes.vlines(ax,  0.25/3, -3, 3, colors='k', linestyles='solid')

dim = len(data[:, 0])
for i in range(0, dim):
    plt.plot([data[i,0],data[i,0]-math.cos(data[i,2])*0.5], [data[i,1], data[i,1]-math.sin(data[i,2])*0.5], '.-', color='blue' )
    point = plt.scatter([data[i,0],data[i,0]-math.cos(data[i,2])*0.5], [data[i,1], data[i,1]-math.sin(data[i,2])*0.5], edgecolors=None)
    plt.pause(0.5)
    point.remove()

plt.show()

