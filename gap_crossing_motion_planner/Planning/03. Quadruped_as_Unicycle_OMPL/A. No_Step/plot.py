from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
import matplotlib.axes
import math
import time

data = numpy.loadtxt('path_example.txt')
W = 0.5

gap_min_x = 1.95
gap_max_x = 3.05
gap_min_y = -1.05
gap_max_y = 1.05

clearance = 0.05

xBR_goal = 4.0
yBR_goal = 0.0
xFR_goal = 5.0
yFR_goal = 0.0
phi_goal = math.atan2(yFR_goal-yBR_goal, xFR_goal-xBR_goal)
xBL_goal = xBR_goal - W * math.sin(phi_goal)
yBL_goal = yBR_goal + W * math.cos(phi_goal)
xFL_goal = xFR_goal - W * math.sin(phi_goal)
yFL_goal = yFR_goal + W * math.cos(phi_goal)


fig = plt.figure()
dim = len(data[:, 0])
BR = data[:, 0:2]
FR = data[:, 2:4]
phi = data[:, 4:5]



BL = []
FL = []
CENTER = []
for i in range(0, dim):
    BL.append([BR[i, 0]-W * math.sin(phi[i]), BR[i, 1]+W * math.cos(phi[i])])
    FL.append([FR[i, 0]-W * math.sin(phi[i]), FR[i, 1]+W * math.cos(phi[i])])
BL = numpy.array(BL)
FL = numpy.array(FL)

for i in range(0, dim):
    CENTER.append([(BR[i, 0] + BL[i, 0] + FR[i, 0] + FL[i, 0])/4, (BR[i, 1] + BL[i, 1] + FR[i, 1] + FL[i, 1])/4])

CENTER = numpy.array(CENTER)


xmax = 5
xmin = -5
ymax = 5
ymin = -5


for i in range(0, dim):
    ax = plt.axes()
    plt.gca().set_aspect('equal')
    plt.xlim(xmin-0.6, xmax+0.6)
    plt.ylim(ymin-0.6, ymax+0.6)

    plt.plot([gap_min_x, gap_max_x],[gap_min_y, gap_min_y], '--', color='black', linewidth=0.5 )
    plt.plot([gap_min_x, gap_max_x],[gap_max_y, gap_max_y], '--', color='black', linewidth=0.5 )
    plt.plot([gap_min_x, gap_min_x],[gap_min_y, gap_max_y], '--', color='black', linewidth=0.5 )
    plt.plot([gap_max_x, gap_max_x],[gap_min_y, gap_max_y], '--', color='black', linewidth=0.5 )

    plt.plot([gap_min_x + clearance, gap_max_x - clearance],[gap_min_y + clearance, gap_min_y + clearance], '.-', color='black', linewidth=0.5 )
    plt.plot([gap_min_x + clearance, gap_max_x - clearance],[gap_max_y - clearance, gap_max_y - clearance], '.-', color='black', linewidth=0.5 )
    plt.plot([gap_min_x + clearance, gap_min_x + clearance],[gap_min_y + clearance, gap_max_y - clearance], '.-', color='black', linewidth=0.5 )
    plt.plot([gap_max_x - clearance, gap_max_x - clearance],[gap_min_y + clearance, gap_max_y - clearance], '.-', color='black', linewidth=0.5 )
    

    plt.plot([xBR_goal, xBL_goal],[yBR_goal, yBL_goal], '.-', color='red', linewidth=2 )
    plt.plot([xBL_goal, xFL_goal],[yBL_goal, yFL_goal], '.-', color='red', linewidth=2 )
    plt.plot([xFL_goal, xFR_goal],[yFL_goal, yFR_goal], '.-', color='red', linewidth=2 )
    plt.plot([xFR_goal, xBR_goal],[yFR_goal, yBR_goal], '.-', color='red', linewidth=2 )
    
##    j = range(0,i)
##    plt.plot([BR[j,0], BL[j,0]],[BR[j, 1], BL[j, 1]], '--', color='blue', linewidth=0.5 )
##    plt.plot([BL[j,0], FL[j,0]],[BL[j, 1], FL[j, 1]], '--', color='blue', linewidth=0.5 )
##    plt.plot([FL[j,0], FR[j,0]],[FL[j, 1], FR[j, 1]], '--', color='blue', linewidth=0.5 )
##    plt.plot([FR[j,0], BR[j,0]],[FR[j, 1], BR[j, 1]], '--', color='blue', linewidth=0.5 )
    
    plt.plot([BR[i,0], BL[i,0]],[BR[i, 1], BL[i, 1]], '.-', color='blue', linewidth=2 )
    plt.plot([BL[i,0], FL[i,0]],[BL[i, 1], FL[i, 1]], '.-', color='blue', linewidth=2 )
    plt.plot([FL[i,0], FR[i,0]],[FL[i, 1], FR[i, 1]], '.-', color='blue', linewidth=2 )
    plt.plot([FR[i,0], BR[i,0]],[FR[i, 1], BR[i, 1]], '.-', color='blue', linewidth=2 )
    
    point = plt.scatter([CENTER[i, 0]],[CENTER[i, 1]], edgecolors=None)
    plt.pause(0.5)
    #point.remove()
    if i != dim-1:
        ax.remove()

plt.show()

