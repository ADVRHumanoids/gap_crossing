from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
import matplotlib.axes
import math
import time

data = numpy.loadtxt('path.txt')
start = [-1.0, -0.25]
goal = [1.0, -0.25]
gap_lenght = 0.1


fig = plt.figure()
ax = plt.axes()

xmax = max(data[:,0])
xmin = min(data[:,0])
ymax = max(data[:,1])
ymin = min(data[:,1])


plt.gca().set_aspect('equal')
plt.xlim(xmin-0.6, xmax+0.6)
plt.ylim(ymin-0.6, ymax+0.6)

y = numpy.linspace(-3, 3, 1000)
matplotlib.axes.Axes.vlines(ax, -gap_lenght/2, ymin-0.6, ymax+0.6, colors='k', linestyles='solid')
matplotlib.axes.Axes.vlines(ax,  gap_lenght/2, ymin-0.6, ymax+0.6, colors='k', linestyles='solid')
matplotlib.axes.Axes.vlines(ax, -gap_lenght/2-0.05, ymin-0.6, ymax+0.6, colors='k', linestyles='dashed')
matplotlib.axes.Axes.vlines(ax,  gap_lenght/2+0.05, ymin-0.6, ymax+0.6, colors='k', linestyles='dashed')

plt.scatter(start[0], start[1], color = 'red')
plt.scatter(goal[0], goal[1], color = 'red')
plt.pause(0.5)

dim = len(data[:, 0])
for i in range(0, dim):
    plt.plot([data[i,0],data[i,0]-math.cos(data[i,2])*0.5], [data[i,1], data[i,1]-math.sin(data[i,2])*0.5], '.-', color='blue' )
    point = plt.scatter([data[i,0],data[i,0]-math.cos(data[i,2])*0.5], [data[i,1], data[i,1]-math.sin(data[i,2])*0.5], edgecolors=None)
    plt.pause(0.5)
    point.remove()

plt.show()

