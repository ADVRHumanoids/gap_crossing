#!/usr/bin/env python  

import numpy as np
import math as m
from matplotlib import path
import time

import rospy
import roslib
import tf
import geometry_msgs.msg

from cartesian_interface.pyci_all import *

#########################################################
######### FUNCTIONS CORRESPONDING TO PRIMITIVES #########
#########################################################

# colors and styles for text to be used as print(color.BOLD + 'Hello World !' + color.END)
class color:
   PURPLE = '\033[95m'
   CYAN = '\033[96m'
   DARKCYAN = '\033[36m'
   BLUE = '\033[94m'
   GREEN = '\033[92m'
   YELLOW = '\033[93m'
   RED = '\033[91m'
   BOLD = '\033[1m'
   UNDERLINE = '\033[4m'
   END = '\033[0m'


# fct giving coords of target_frame wrt reference_frame
def getPose(reference_frame, target_frame):
    listener = tf.TransformListener()
    listener.waitForTransform(reference_frame, target_frame, rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
            now = rospy.Time(0)
            listener.waitForTransform(reference_frame, target_frame, now, rospy.Duration(4.0))
            (t,r) = listener.lookupTransform(reference_frame, target_frame, now)
            return (t, r)   
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


# fct for rolling along the x axis and y axis
def roll(task, distance, axis, duration, to_be_disabled):
    
    for item in to_be_disabled:
        item.disable()
    
    pose, _, _ = task.getPoseReference()
    R = pose.linear_ref()
    c = R[0, 0]
    s = R[1, 0]

    if axis == 'x':
        pose.translation_ref()[0] += c*distance
        pose.translation_ref()[1] += s*distance
    elif axis == 'y':
        pose.translation_ref()[0] += s*distance
        pose.translation_ref()[1] += -c*distance
    
    task.setPoseTarget(pose, duration)
    task.waitReachCompleted(1.5*duration)
    
    for item in to_be_disabled:
        item.enable()


def roll2(task, distance, axis, duration, to_be_disabled):

    for item in to_be_disabled:
        item.disable()
    
    pose, _, _ = task.getPoseReference()
    
    x_init = pose.translation[0]
    y_init = pose.translation[1]
    x_goal = 0.
    y_goal = 0.
    
    R = pose.linear_ref()
    c = R[0, 0]
    s = R[1, 0]

    t = 0.0
    dt = 0.01 

    if axis == 'x':
        x_goal = x_init + c*distance
        y_goal = y_init + s*distance
    elif axis == 'y':
        x_goal = x_init + s*distance
        y_goal = y_init - c*distance
    
    a = 2 * (x_init - x_goal)/duration**3
    b = -3 * (x_init - x_goal)/duration**2
    c = 0
    d = x_init

    alpha = 2 * (y_init - y_goal)/duration**3
    beta = -3 * (y_init - y_goal)/duration**2
    gamma = 0
    delta = y_init
    
    while t < duration:
        pose.translation_ref()[0] = a * t**3 + b * t**2 + c * t + d
        pose.translation_ref()[1] = alpha * t**3 + beta * t**2 + gamma * t + delta

        task.setPoseReference(pose) # this publishes the reference

        t += dt
        time.sleep(dt)

    for item in to_be_disabled:
        item.enable()

   

# fct for spinning (i.e., rotating around the center of the sp -> rotation about the z-axis)
def spin(task, angle, duration, to_be_disabled):

    for item in to_be_disabled:
        item.disable()

    pose, _, _ = task.getPoseReference()
    rot = np.array([[m.cos(angle), -m.sin(angle), 0.],[m.sin(angle), m.cos(angle), 0.],[0., 0., 1.]])
    temp = (pose.linear_ref()).dot(rot)
    
    for row in range(0, 3):
        for col in range(0, 3):
            pose.linear_ref()[row][col] = temp[row][col]
            
    task.setPoseTarget(pose, duration)
    task.waitReachCompleted(1.5*duration)
    
    for item in to_be_disabled:
        item.enable()


# fct for moving the com keeping the same stance. "goal" is a variation wrt the car frame
def com_translation(car, com, goal, duration, to_be_disabled):
    
    for item in to_be_disabled:
        item.disable()

    p_w_com, q_w_com = getPose('ci/world', 'ci/com')
    p_w_car, q_w_car = getPose('ci/world', 'ci/car_frame')
    p_car_com, _ = getPose('ci/car_frame', 'ci/com')

    Tcar, _, _ = car.getPoseReference()
    Tcom, _, _ = com.getPoseReference()

    Tcar.translation = p_w_car
    Tcar.quaternion = q_w_car
    
    transform_w_car = Tcar.matrix()
    
    p_car_com += goal
    p_car_com = np.concatenate((p_car_com, np.array([1])), axis=0)

    p_w_com = (transform_w_car).dot(p_car_com.T)
    
    for i in range(0, 3):
        Tcom.translation_ref()[i] = p_w_com[i]
    com.setPoseTarget(Tcom, duration)
    com.waitReachCompleted(1.5*duration)

    for item in to_be_disabled:
        item.enable()
        

# fct for stepping. the step is a semi-circumference. every 0.1s, it is checked if the com lies in the support triangle
def step(moving_foot, still_feet, step_length, duration, to_be_disabled_step):
    
    for item in to_be_disabled_step:
        item.disable()

    t = 0.0
    dt = 0.01
    period = 2*duration
    radius = step_length/2

    moving_foot_pose, _, _ = moving_foot.getPoseReference()

    while t < duration:
        
        delta_x = radius - radius * m.cos(t*2.0*m.pi/period)
        delta_z = radius * m.sin(t*2.0*m.pi/period)
        Tref = moving_foot_pose.copy()
        Tref.translation_ref()[0] += delta_x
        Tref.translation_ref()[2] += delta_z
        
        moving_foot.setPoseReference(Tref) # this publishes the reference
        
        t += dt
        time.sleep(dt)

        if t%10 == 0:
            p_car_f0, _ = getPose('ci/car_frame', 'ci/'+still_feet[0].getDistalLink())
            p_car_f1, _ = getPose('ci/car_frame', 'ci/'+still_feet[1].getDistalLink())
            p_car_f2, _ = getPose('ci/car_frame', 'ci/'+still_feet[2].getDistalLink())
            p_car_com, _ = getPose('ci/car_frame', 'ci/com')

            sp = path.Path([(p_car_f0[0], p_car_f0[1]), (p_car_f1[0], p_car_f1[1]), (p_car_f2[0], p_car_f2[1])])
            flag = sp.contains_points([(p_car_com[0], p_car_com[1])])
            if flag == False:
                print "\n\n :( \n\n"

    for item in to_be_disabled_step:
        item.enable()    
    


#########################################################
########################## MAIN #########################
#########################################################

def main():
    # load file with primitives and number of times they are applied
    temp = np.loadtxt('/home/matteo/catkin_ws/src/centauro_cartesio-devel-cpack/python/temp.txt')
    index = int(temp)
    
    plan = np.loadtxt('/home/matteo/catkin_ws/src/centauro_cartesio-devel-cpack/python/try_roll.txt')
    primitive = plan[index, 0]
    application = plan[index, 1]

    rospy.init_node('ci')                   # needed for the listener

    cli = pyci.CartesianInterfaceRos()      # initialization of cartesIO

    # tasks
    com = cli.getTask('Com')
    car = cli.getTask('car_frame')
    pelvis = cli.getTask('pelvis')
    w1 = cli.getTask('wheel_1')
    w2 = cli.getTask('wheel_2')
    w3 = cli.getTask('wheel_3')
    w4 = cli.getTask('wheel_4')
      
    if primitive == 0:
        print(color.BOLD + 'Primitive 0: clockwise spin of 10 deg for ' + str(int(application)) + ' times. (' + str(i) + '/' + str(n_primitives) + ')' + color.END)
        spin(car, -m.pi/18 * application, 5.0 * application, [com, pelvis])
    elif primitive == 1:
        print(color.BOLD + 'Primitive 1: counter-clockwise spin of 10 deg for ' + str(int(application)) + ' times. (' + str(i) + '/' + str(n_primitives) + ')' + color.END)
        spin(car, m.pi/18 * application, 5.0 * application, [com, pelvis])
    elif primitive == 2:
        print(color.BOLD + 'Primitive 2: forward roll of 0.05 cm for ' + str(int(application)) + ' times. (' + str(i) + '/' + str(n_primitives) + ')' + color.END)
        roll(car, 0.05 * application, 'x', 1.0 * application, [com, pelvis])
    elif primitive == 3:
        print(color.BOLD + 'Primitive 3: backward roll of 0.05 cm for ' + str(int(application)) + ' times. (' + str(i) + '/' + str(n_primitives) + ')' + color.END)
        roll(car, -0.05 * application, 'x', 1.0 * application, [com, pelvis])
    elif primitive == 4:
        print(color.BOLD + 'Primitive 4: right roll of 0.05 cm for ' + str(int(application)) + ' times. (' + str(i) + '/' + str(n_primitives) + ')' + color.END)
        roll(car, 0.05 * application, 'y', 1.0 * application, [com, pelvis])
    elif primitive == 5:
        print(color.BOLD + 'Primitive 5: left roll of 0.05 cm for ' + str(int(application)) + ' times. (' + str(i) + '/' + str(n_primitives) + ')' + color.END)
        roll(car, -0.05 * application, 'y', 1.0 * application, [com, pelvis])
    elif primitive == 6:
        print(color.BOLD + 'Primitive 6: step of 0.20 cm with BR foot. (' + str(i) + '/' + str(n_primitives) + ')' + color.END)
        step(moving_foot = w4, still_feet = [w1, w2, w3], step_length = 0.2, duration = 4.0, to_be_disabled_step = [pelvis, com])
    elif primitive == 7:
        print(color.BOLD + 'Primitive 7: step of 0.20 cm with BL foot. (' + str(i) + '/' + str(n_primitives) + ')' + color.END)
        step(moving_foot = w3, still_feet = [w1, w2, w4], step_length = 0.2, duration = 4.0, to_be_disabled_step = [pelvis, com])
    elif primitive == 8:
        print(color.BOLD + 'Primitive 8: step of 0.20 cm with FR foot. (' + str(i) + '/' + str(n_primitives) + ')' + color.END)
        step(moving_foot = w2, still_feet = [w1, w3, w4], step_length = 0.2, duration = 4.0, to_be_disabled_step = [pelvis, com])
    elif primitive == 9:
        print(color.BOLD + 'Primitive 9: step of 0.20 cm with FL foot. (' + str(i) + '/' + str(n_primitives) + ')' + color.END)
        step(moving_foot = w1, still_feet = [w2, w3, w4], step_length = 0.2, duration = 4.0, to_be_disabled_step = [pelvis, com])

    cli.update()
    '''   
    f1, _ = getPose('ci/world', 'ci/wheel_1')
    f2, _ = getPose('ci/world', 'ci/wheel_2')
    f3, _ = getPose('ci/world', 'ci/wheel_3')
    f4, _ = getPose('ci/world', 'ci/wheel_4')

    f1 = f1[0:2]
    f2 = f2[0:2]
    f3 = f3[0:2]
    f4 = f4[0:2]

    w1 = plan[i, 6:8]
    w2 = plan[i, 4:6]
    w3 = plan[i, 2:4]
    w4 = plan[i, 0:2]
         
    print(color.PURPLE + 'Errors: ' + color.END)
    print('wheel_1: ' + str(w1 - f1))
    print('wheel_2: ' + str(w2 - f2))
    print('wheel_3: ' + str(w3 - f3))
    print('wheel_4: ' + str(w4 - f4))
    '''    
    time.sleep(5.0)
    
# main
if __name__ == "__main__":
    main()
