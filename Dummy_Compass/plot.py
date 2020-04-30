from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
import matplotlib.axes
import math
import time

"""
#define CW_AROUND_R  0 //clockwise rotation around right foot of DELTA_ROT
#define CCW_AROUND_R 1 //counter-clockwise rotation around right foot of DELTA_ROT
#define CW_AROUND_L  2 //clockwise rotation around left foot of DELTA_ROT
#define CCW_AROUND_L 3 //counter-clockwise rotation around left foot of DELTA_ROT
//Short steps
#define SH_FW_R 4 //short forward step with right foot of SHORT_STEP
#define SH_BW_R 5 //short backward step with right foot of SHORT_STEP
#define SH_FW_L 6 //short forward step with left foot of SHORT_STEP
#define SH_BW_L 7 //short backward step with left foot of SHORT_STEP
//Long steps
#define LO_FW_R 8 //long forward step with right foot of LONG_STEP
#define LO_FW_L 9 //long forward step with left foot of LONG_STEP
//Lateral step
#define R_LAT_R 10 //right foot moves to right of LATERAL_STEP
#define L_LAT_R 11 //right foot moves to left of LATERAL_STEP
#define R_LAT_L 12 //left foot moves to right of LATERAL_STEP
#define L_LAT_L 13 //left foot moves to left of LATERAL_STEP
"""

def control2verb(ctrl):
    if ctrl == 0:
        string = 'clockwise rotation around right foot of DELTA_ROT'
    elif ctrl == 1:
        string = 'counter-clockwise rotation around right foot of DELTA_ROT'
    elif ctrl == 2:
        string = 'clockwise rotation around left foot of DELTA_ROT'
    elif ctrl == 3:
        string = 'counter-clockwise rotation around left foot of DELTA_ROT'
    elif ctrl == 4:
        string = 'short forward step with right foot of SHORT_STEP'
    elif ctrl == 5:
        string = 'short backward step with right foot of SHORT_STEP'
    elif ctrl == 6:
        string = 'short forward step with left foot of SHORT_STEP'
    elif ctrl == 7:
        string = 'short backward step with left foot of SHORT_STEP'
    elif ctrl == 8:
        string = 'long forward step with right foot of SHORT_STEP'
    elif ctrl == 9:
        string = 'long forward step with left foot of SHORT_STEP'
    elif ctrl == 10:
        string = 'right foot moves to right of LATERAL_STEP'
    elif ctrl == 11:
        string = 'right foot moves to left of LATERAL_STEP'
    elif ctrl == 12:
        string = 'left foot moves to right of LATERAL_STEP'
    elif ctrl == 13:
        string = 'left foot moves to left of LATERAL_STEP'
    else:
        string = 'ERROR'
    return string


data = numpy.loadtxt('path.txt')

R = data[:, 0:2]
L = data[:, 2:4]
Control = data[:, 5]

Rstart = [-1.0, -0.5]
Lstart = [-1.0, 0.5]

Rgoal = [2.0, 0.0]
Lgoal = [3.0, 0.0]

fig = plt.figure()
ax = plt.axes()

xmax = max(max(R[:, 0]), max(L[:, 0]))
xmin = min(min(R[:, 0]), min(L[:, 0]))
ymax = max(max(R[:, 1]), max(L[:, 1]))
ymin = min(min(R[:, 1]), min(L[:, 1]))

margin = 0.5
plt.gca().set_aspect('equal')
plt.xlim(xmin-margin, xmax+margin)
plt.ylim(ymin-margin, ymax+margin)

gaplength = 0.1
obstacle_vertex = [[1.0, 0.5], [1.5, 0.5], [1.0, -0.5], [1.5, -0.5]]

matplotlib.axes.Axes.vlines(ax, -gaplength/2, ymin-margin, ymax+margin, colors='k', linestyles='dashed', linewidth=2)
matplotlib.axes.Axes.vlines(ax, gaplength/2, ymin-margin, ymax+margin, colors='k', linestyles='dashed', linewidth=2)

plt.scatter([Rstart[0], Lstart[0]], [Rstart[1], Lstart[1]], color='red')
plt.scatter([Rgoal[0], Lgoal[0]], [Rgoal[1], Lgoal[1]], color='red')

plt.plot([1.0, 1.5],[0.5, 0.5], '.-', color='black', linewidth=2)
plt.plot([1.5, 1.5],[0.5, -0.5], '.-', color='black', linewidth=2)
plt.plot([1.5, 1.0],[-0.5, -0.5], '.-', color='black', linewidth=2)
plt.plot([1.0, 1.0],[-0.5, 0.5], '.-', color='black', linewidth=2)

plt.pause(0.5)

dim = len(data[:, 0])

r = 0
l = 0

for i in range(0, dim):
    ctrl = int(Control[i])
    string = control2verb(ctrl)
    ax.set_title(string, fontdict={'fontsize':30})
    plt.plot([R[i, 0], L[i, 0]], [R[i, 1], L[i, 1]], '.-', color='blue', linewidth=2)
    if i==0:
        stringR = 'R'+str(i)
        plt.text(R[i, 0], R[i, 1], stringR)
        stringL = 'L'+str(i)
        plt.text(L[i, 0], L[i, 1], stringL)
    elif i > 0 and R[i, 0] != R[i-1, 0] or R[i, 1] != R[i-1, 1]:
        r = r+1
        stringR = 'R'+str(r)
        plt.text(R[i, 0], R[i, 1], stringR)
    elif i > 0 and L[i, 0] != L[i-1, 0] or L[i, 1] != L[i-1, 1]:
        l = l+1
        stringL = 'L'+str(l)
        plt.text(L[i, 0], L[i, 1], stringL)
    point = plt.scatter([R[i, 0], L[i, 0]], [R[i, 1], L[i, 1]], edgecolors=None)
    plt.pause(2)
    point.remove()

ax.set_title("")
plt.show()
