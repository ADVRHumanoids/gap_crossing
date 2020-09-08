from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches
import numpy
import matplotlib.pyplot as plt
import matplotlib.axes
import math
import time




def control2verbose(ctrl):
    if ctrl == 0:
        string = "clockwise spin around the geometric center of the support polygon"
    elif ctrl == 1:
        string = "counter-clockwise spin around the geometric center of the support polygon"
    elif ctrl == 2:
        string = "forward roll of the four wheels"
    elif ctrl == 3:
        string = "backward roll of the four wheels"
    elif ctrl == 4:
        string = "right roll of the four wheels"
    elif ctrl == 5:
        string = "left roll of the four wheels"
    elif ctrl == 6:
        string = "forward step of back right foot"
    elif ctrl == 7:
        string = "forward step of back left foot"
    elif ctrl == 8:
        string = "forward step of front right foot"
    elif ctrl == 9:
        string = "forward step of front left foot"
    return string


data = numpy.loadtxt('path_example.txt')

xBR_goal = 2.5
yBR_goal = -0.35
xBL_goal = 2.5
yBL_goal = 0.35
xFR_goal = 3.2
yFR_goal = -0.35
xFL_goal = 3.2
yFL_goal = 0.35

xCOMB_goal = (xBR_goal + xBL_goal)/2
yCOMB_goal = (yBR_goal + yBL_goal)/2
xCOMF_goal = (xFR_goal + xFL_goal)/2
yCOMF_goal = (yFR_goal + yFL_goal)/2

x_gap_min = 2.215
x_gap_max = 2.365
y_gap_min = -1.55
y_gap_max = 0.35

activation_step = 0.05

x_obs_min_true = 0.75
x_obs_max_true = 1.56
y_obs_min_true = -0.15
y_obs_max_true = 0.35

x_obs_min_safe = 0.65
x_obs_max_safe = 1.65
y_obs_min_safe = -0.35
y_obs_max_safe = 0.35

fig = plt.figure()
dim = len(data[:, 0])
BR = data[:, 0:2]
BL = data[:, 2:4]
COMB = (BR+BL)/2
FR = data[:, 4:6]
FL = data[:, 6:8]
COMF = (FR+FL)/2
Control = data[:, 8:9]


CENTER = []

for i in range(0, dim):
    CENTER.append([(BR[i, 0] + BL[i, 0] + FR[i, 0] + FL[i, 0])/4, (BR[i, 1] + BL[i, 1] + FR[i, 1] + FL[i, 1])/4])

CENTER = numpy.array(CENTER)



xmax = max(numpy.hstack((BR[:, 0], BL[:, 0], FR[:, 0], FL[:, 0], x_gap_min, x_gap_max)))
xmin = min(numpy.hstack((BR[:, 0], BL[:, 0], FR[:, 0], FL[:, 0], x_gap_min, x_gap_max)))
ymax = max(numpy.hstack((BR[:, 1], BL[:, 1], FR[:, 1], FL[:, 1], y_gap_min, y_gap_max)))
ymin = min(numpy.hstack((BR[:, 1], BL[:, 1], FR[:, 1], FL[:, 1], y_gap_min, y_gap_max)))


green = [200./255, 247./255, 197./255]
red = [1., 0., 0.]
pink = [255./255, 182./255, 193./255]
for i in range(0, dim):
    ax = plt.axes()
    plt.gca().set_aspect('equal')
    plt.xlim(xmin-0.5, xmax+0.5)
    plt.ylim(ymin-0.5, ymax+0.5)

    activation_step_area = matplotlib.patches.Polygon(numpy.array([[x_gap_min-activation_step, y_gap_min], [x_gap_min, y_gap_min], [x_gap_min, y_gap_max], [x_gap_min-activation_step, y_gap_max]]), facecolor = green, edgecolor=green )
    ax.add_patch(activation_step_area)
    
    point = plt.scatter([CENTER[i, 0]],[CENTER[i, 1]], edgecolors=None)

    gap = matplotlib.patches.Polygon(numpy.array([[x_gap_min, y_gap_min], [x_gap_max, y_gap_min], [x_gap_max, y_gap_max], [x_gap_min, y_gap_max]]), hatch = '/', facecolor = 'white', edgecolor = 'black' )
    ax.add_patch(gap)

    obstacle_safe = matplotlib.patches.Polygon(numpy.array([[x_obs_min_safe, y_obs_min_safe], [x_obs_min_safe, y_obs_max_safe], [x_obs_max_safe, y_obs_max_safe],
                                                            [x_obs_max_safe, y_obs_min_safe]]), facecolor = pink, edgecolor=pink )
    ax.add_patch(obstacle_safe)
    
    obstacle_true = matplotlib.patches.Polygon(numpy.array([[x_obs_min_true, y_obs_min_true], [x_obs_min_true, y_obs_max_true], [x_obs_max_true, y_obs_max_true],
                                                            [x_obs_max_true, y_obs_min_true]]), facecolor = red, edgecolor=red )
    ax.add_patch(obstacle_true)
    
    
    plt.plot([xBR_goal, xBL_goal],[yBR_goal, yBL_goal], '.-', color='red', linewidth=2 )
    plt.plot([xFL_goal, xFR_goal],[yFL_goal, yFR_goal], '.-', color='red', linewidth=2 )
    plt.plot([xCOMB_goal, xCOMF_goal],[yCOMB_goal, yCOMF_goal], '.-', color='red', linewidth=2 )
    
    
    plt.plot([BR[i,0], BL[i,0]],[BR[i, 1], BL[i, 1]], '.-', color='blue', linewidth=2 )
    plt.plot([FL[i,0], FR[i,0]],[FL[i, 1], FR[i, 1]], '.-', color='blue', linewidth=2 )
    plt.plot([COMB[i,0], COMF[i,0]],[COMB[i, 1], COMF[i, 1]], '.-', color='blue', linewidth=2 )

    if i == 0:
        plt.pause(0.5)
    else:
        ax.set_title(control2verbose(int(Control[i])))
        plt.pause(0.5)
    #point.remove()
    if i != dim-1:
        ax.remove()
ax.set_title("")
plt.show()

'''
i = 0
ax = plt.axes()
plt.gca().set_aspect('equal')
plt.xlim(xmin-0.5, xmax+0.5)
plt.ylim(ymin-0.5, ymax+0.5)
point = plt.scatter([CENTER[i, 0]],[CENTER[i, 1]], edgecolors=None)

activation_step_area = matplotlib.patches.Polygon(numpy.array([[x_gap_min-activation_step, y_gap_min], [x_gap_min, y_gap_min], [x_gap_min, y_gap_max], [x_gap_min-activation_step, y_gap_max]]), facecolor = green, edgecolor=green )
ax.add_patch(activation_step_area)

gap = matplotlib.patches.Polygon(numpy.array([[x_gap_min, y_gap_min], [x_gap_max, y_gap_min], [x_gap_max, y_gap_max], [x_gap_min, y_gap_max]]), hatch = '/', facecolor = 'white', edgecolor = 'black' )
ax.add_patch(gap)

plt.plot([xBR_goal, xBL_goal],[yBR_goal, yBL_goal], '.-', color='red', linewidth=2 )
plt.plot([xFL_goal, xFR_goal],[yFL_goal, yFR_goal], '.-', color='red', linewidth=2 )
plt.plot([xCOMB_goal, xCOMF_goal],[yCOMB_goal, yCOMF_goal], '.-', color='red', linewidth=2 )


plt.plot([BR[i,0], BL[i,0]],[BR[i, 1], BL[i, 1]], '.-', color='blue', linewidth=2 )
plt.plot([FL[i,0], FR[i,0]],[FL[i, 1], FR[i, 1]], '.-', color='blue', linewidth=2 )
plt.plot([COMB[i,0], COMF[i,0]],[COMB[i, 1], COMF[i, 1]], '.-', color='blue', linewidth=2 )

    
ax.set_title("")
plt.savefig("init3.png")
'''
