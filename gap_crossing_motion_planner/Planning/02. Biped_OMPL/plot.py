from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
import matplotlib.axes
import math
import time


"""
#define CW_AROUND_R  	0 //cloak-wise rotation around right foot of DELTA_ROT
#define CCW_AROUND_R 	1 //countercloak-wise rotation around right foot of DELTA_ROT
#define CW_AROUND_L  	2 //cloak-wise rotation around left foot of DELTA_ROT
#define CCW_AROUND_L 	3 //countercloak-wise rotation around left foot of DELTA_ROT
//Short steps
#define SH_FW_R 		4 //short forward step with right foot of SHORT_STEP
#define SH_BW_R 		5 //short backward step with right foot of SHORT_STEP
#define SH_FW_L 		6 //short forward step with left foot of SHORT_STEP
#define SH_BW_L 		7 //short backward step with left foot of SHORT_STEP
//Long steps
#define LO_FW_R 		8 //long forward step with right foot of LONG_STEP
#define LO_FW_L 		9 //long forward step with left foot of LONG_STEP
//Lateral step
#define R_LAT_R 		10 //right foot moves to right of LATERAL_STEP
#define L_LAT_R 		11 //right foot moves to left of LATERAL_STEP
#define R_LAT_L 		12 //left foot moves to right of LATERAL_STEP
#define L_LAT_L 		13 //left foot moves to left of LATERAL_STEP
"""

def control2verbose(ctrl):
    if ctrl == 0:
        string = "clock-wise rotation around right foot"
    elif ctrl == 1:
        string = "counterclock-wise rotation around right foot"
    elif ctrl == 2:
        string = "clock-wise rotation around left foot"
    elif ctrl == 3:
        string = "counterclock-wise rotation around left foot"
    elif ctrl == 4:
        string = "short forward step with right foot"
    elif ctrl == 5:
        string = "short backward step with right foot"
    elif ctrl == 6:
        string = "short forward step with left foot"
    elif ctrl == 7:
        string = "short backward step with left foot"
    elif ctrl == 8:
        string = "long forward step with right foot"
    elif ctrl == 9:
        string = "long forward step with left foot"
    elif ctrl == 10:
        string = "lateral step with right foot to right"
    elif ctrl == 11:
        string = "lateral step with right foot to left"
    elif ctrl == 12:
        string = "lateral step with left foot to right"
    elif ctrl == 13:
        string = "lateral step with left foot to left" 
    return string
    


data = numpy.loadtxt('path_example.txt')

R = data[:, 0:2]
L = data[:, 2:4]
Control = data[:, 5:6]

R_start = [-1.0, -0.5]
L_start = [-1.0, 0.5]

R_goal = [2.0, 0.0]
L_goal = [3.0, 0.0]
goal = [1.0, -0.25]

gap_lenght = 0.1

obs_vertices = [[1.0, 0.5], [1.5, 0.5], [1.5, -0.5], [1.0, -0.5]]


fig = plt.figure()
ax = plt.axes()


X = numpy.concatenate((R[:,0], L[:,0]),axis=0)
Y = numpy.concatenate((R[:,1], L[:,1]),axis=0)

xmax = max(X)
xmin = min(X)
ymax = max(Y)
ymin = min(Y)


plt.gca().set_aspect('equal')
plt.xlim(xmin-0.6, xmax+0.6)
plt.ylim(ymin-0.6, ymax+0.6)


matplotlib.axes.Axes.vlines(ax, -gap_lenght/2, ymin-0.6, ymax+0.6, colors='k', linestyles='solid')
matplotlib.axes.Axes.vlines(ax,  gap_lenght/2, ymin-0.6, ymax+0.6, colors='k', linestyles='solid')

num_vertices = len(obs_vertices)
for i in range(num_vertices):
    next_vertex = (i+1)%num_vertices
    plt.plot([obs_vertices[i][0],obs_vertices[next_vertex][0]], [obs_vertices[i][1], obs_vertices[next_vertex][1]], '.-', color = 'black')


plt.scatter([R_start[0], L_start[0]], [R_start[1], L_start[1]], color = 'red')
plt.scatter([R_goal[0], L_goal[0]], [R_goal[1], L_goal[1]], color = 'red')
#plt.pause(0.5)

dim = len(data[:, 0])

r = 0;
l = 0;

for i in range(0, dim):
    plt.pause(0.6)
    ax.set_title(control2verbose(int(Control[i])))
    plt.plot([R[i,0], L[i,0]],[R[i, 1], L[i, 1]], '.-', color='blue' )
    if i == 0:
        stringR = 'R' + str(r)
        plt.text(R[i, 0],R[i, 1],stringR)
        stringL = 'L' + str(l)
        plt.text(L[i, 0],L[i, 1],stringL)
    elif i > 0 and R[i, 0] != R[i-1, 0] or R[i, 1] != R[i-1, 1]:
        r = r + 1
        stringR = 'R' + str(r)
        plt.text(R[i, 0],R[i, 1],stringR)      
    elif i > 0 and L[i, 0] != L[i-1, 0] or L[i, 1] != L[i-1, 1]:
        l = l + 1
        stringL = 'L' + str(l)
        plt.text(L[i, 0],L[i, 1],stringL)
    #point = plt.scatter([R[i,0], L[i,0]],[R[i, 1], L[i, 1]], edgecolors=None)
    
    #point.remove()
ax.set_title("")
plt.show()

