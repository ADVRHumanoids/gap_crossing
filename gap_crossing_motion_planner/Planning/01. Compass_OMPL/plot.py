from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
import matplotlib.axes
import math
import time

data = numpy.loadtxt('path_example.txt')

start = [-2.0, -0.15]
goal = [2.0, -0.15]
gap_lenght = 0.1


fig = plt.figure()
ax = plt.axes()

xmax = 2.5
xmin = -2.5
ymax = 2.5
ymin = -2.5


plt.gca().set_aspect('equal')
plt.xlim(xmin, xmax)
plt.ylim(ymin, ymax)

y = numpy.linspace(-3, 3, 1000)
matplotlib.axes.Axes.vlines(ax, 0.75-gap_lenght/2, ymin, ymax, colors='k', linestyles='solid')
matplotlib.axes.Axes.vlines(ax, 0.75+gap_lenght/2, ymin, ymax, colors='k', linestyles='solid')

plt.scatter(start[0], start[1], color = 'red')
plt.scatter(goal[0], goal[1], color = 'red')
plt.pause(0.5)

dim = len(data[:, 0])
R = data[:, 0:2]
theta = data[:, 2:3]
L = []
for i in range(0, dim):
    L.append([R[i, 0]-math.cos(theta[i])*0.3, R[i, 1]-math.sin(theta[i])*0.3])
L = numpy.array(L)
r = 0
l = 0

red = [0.5, 0., 0.]
for i in range(0, dim):
    plt.plot([R[i,0], L[i,0]],[R[i, 1], L[i, 1]], '.-', color='blue' )
 
    obstacle = matplotlib.patches.Polygon(numpy.array([[-1, -1], [-1, 1], [-0.5, 1],
                                                            [-0.5, -1]]), facecolor = red, edgecolor=red )
    ax.add_patch(obstacle)
    
    if i == 0:
        stringR = 'R' + str(i)
        plt.text(R[i, 0],R[i, 1],stringR)
        stringL = 'L' + str(i)
        plt.text(L[i, 0],L[i, 1],stringL)
    elif i > 0 and R[i, 0]!= R[i-1, 0] or R[i, 1] != R[i-1, 1]:
        r = r +1
        stringR = 'R' + str(r)
        plt.text(R[i, 0],R[i, 1],stringR)      
    elif i > 0 and L[i, 0]!= L[i-1, 0] or L[i, 1] != L[i-1, 1]:
        l = l + 1
        stringL = 'L' + str(l)
        if l == 5:
            plt.text(L[i, 0]-0.1,L[i, 1]-0.1,stringL)
        else:
            plt.text(L[i, 0],L[i, 1],stringL)
    point = plt.scatter([R[i,0], L[i,0]],[R[i, 1], L[i, 1]], edgecolors=None)
    plt.pause(0.5)
    point.remove()

plt.show()

