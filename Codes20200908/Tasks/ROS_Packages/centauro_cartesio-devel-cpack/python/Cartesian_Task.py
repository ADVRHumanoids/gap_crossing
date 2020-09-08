#!/usr/bin/env python  

import numpy as np
import math as m
from matplotlib import path
from numpy.linalg import inv
import time

import rospy
import roslib
import tf
import geometry_msgs.msg

from cartesian_interface.pyci_all import *

#########################################################
############ AUXILIARY FUNCTIONS AND CLASSES ############
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

# computes coefficients of a cubic with given initial and final positions and 0 velocity at beginning and end
def cubic(vi, vf, dvi, dvf, duration):
    a = ((dvi + dvf)*duration + 2*(vi-vf))/duration**3
    b = (-(2*dvi + dvf)*duration + 3*(vf-vi))/duration**2
    c = dvi
    d = vi
    return a, b, c, d

   
#########################################################
############## BRING TO HOME CONFIGURATION ##############
#########################################################

def homing(car, w1, w2, w3, w4, pelvis, com):
    com.disable()
    Tw1, _, _ = w1.getPoseReference()
    Tw2, _, _ = w2.getPoseReference()
    Tw3, _, _ = w3.getPoseReference()
    Tw4, _, _ = w4.getPoseReference()
    Tcar, _, _ = car.getPoseReference()

    Tw1.translation_ref()[0] = 0.35
    Tw1.translation_ref()[1] = 0.35
    Tw2.translation_ref()[0] = 0.35
    Tw2.translation_ref()[1] = -0.35
    Tw3.translation_ref()[0] = -0.35
    Tw3.translation_ref()[1] = 0.35
    Tw4.translation_ref()[0] = -0.35
    Tw4.translation_ref()[1] = -0.35

    w1.setPoseTarget(Tw1, 1.0)
    w2.setPoseTarget(Tw2, 1.0)
    w3.setPoseTarget(Tw3, 1.0)
    w4.setPoseTarget(Tw4, 1.0)

    w1.waitReachCompleted(1.5)
    w2.waitReachCompleted(1.5)
    w3.waitReachCompleted(1.5)
    w4.waitReachCompleted(1.5)

    com.enable()
    
    movecar(car, [w1, w2, w3, w4], 1.0, [pelvis, com])
    

#########################################################
######### FUNCTIONS CORRESPONDING TO PRIMITIVES #########
#########################################################

# fct for rolling along the x axis and y axis
def roll(car, wheels, distance, axis, duration, to_be_disabled, cli):

    for item in to_be_disabled:
        item.disable()
    
    Tcar = cli.getPoseFromTf('ci/car_frame', 'ci/world')

    Tw1 = cli.getPoseFromTf('ci/'+wheels[0].getDistalLink(), 'ci/car_frame')
    Tw2 = cli.getPoseFromTf('ci/'+wheels[1].getDistalLink(), 'ci/car_frame')
    Tw3 = cli.getPoseFromTf('ci/'+wheels[2].getDistalLink(), 'ci/car_frame')
    Tw4 = cli.getPoseFromTf('ci/'+wheels[3].getDistalLink(), 'ci/car_frame')
    
    x_init = Tcar.translation[0]
    y_init = Tcar.translation[1]
    
    R = Tcar.linear
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

    a_x, b_x, c_x, d_x = cubic(x_init, x_goal, 0., 0., duration)
    a_y, b_y, c_y, d_y = cubic(y_init, y_goal, 0., 0., duration)
    
    while t < duration:
        Tcar.translation_ref()[0] = a_x * t**3 + b_x * t**2 + c_x * t + d_x
        Tcar.translation_ref()[1] = a_y * t**3 + b_y * t**2 + c_y * t + d_y

        car.setPoseReference(Tcar)           # this publishes the reference
        wheels[0].setPoseReference(Tw1)
        wheels[1].setPoseReference(Tw2)
        wheels[2].setPoseReference(Tw3)
        wheels[3].setPoseReference(Tw4)

        t += dt
        time.sleep(dt)

    for item in to_be_disabled:
        item.enable()


# fct for rolling two wheels only
def rollTwoWheelsandMoveCom(wheels, distance_wheels, com, distance_com, movecom, other_wheels, duration, to_be_disabled, cli):

    for item in to_be_disabled:
        item.disable()

    if movecom:
        com.enable()   
    
    Tw0 = cli.getPoseFromTf('ci/'+wheels[0].getDistalLink(), 'ci/car_frame')
    Tw1 = cli.getPoseFromTf('ci/'+wheels[1].getDistalLink(), 'ci/car_frame')
    Tcom = cli.getPoseFromTf('ci/'+com.getDistalLink(), 'ci/world')

    other_wheels[0].setBaseLink(u'world')
    other_wheels[1].setBaseLink(u'world')
    Tw2 = cli.getPoseFromTf('ci/'+other_wheels[0].getDistalLink(), 'ci/world')
    Tw3 = cli.getPoseFromTf('ci/'+other_wheels[1].getDistalLink(), 'ci/world')

    Tw0_trans = Tw0.translation
    Tw1_trans = Tw1.translation
    Tcom_trans = Tcom.translation
    Tw2_trans = Tw2.translation
    Tw3_trans = Tw3.translation

    a_x_w0, b_x_w0, c_x_w0, d_x_w0 = cubic(Tw0_trans[0], Tw0_trans[0] + distance_wheels, 0., 0., duration)
    a_x_w1, b_x_w1, c_x_w1, d_x_w1 = cubic(Tw1_trans[0], Tw1_trans[0] + distance_wheels, 0., 0., duration)
    a_x_com, b_x_com, c_x_com, d_x_com = cubic(Tcom_trans[0], Tcom_trans[0] + distance_com, 0., 0., duration)

    t = 0.0
    dt = 0.01
    
    while t < duration:
        Tw0.translation_ref()[0] = a_x_w0 * t**3 + b_x_w0 * t**2 + c_x_w0 * t + d_x_w0
        Tw1.translation_ref()[0] = a_x_w1 * t**3 + b_x_w1 * t**2 + c_x_w1 * t + d_x_w1
        Tw2.translation_ref()[0] = Tw2.translation[0]
        Tw3.translation_ref()[0] = Tw3.translation[0]
        Tw2.translation_ref()[1] = Tw2.translation[1]
        Tw3.translation_ref()[1] = Tw3.translation[1]
        Tw2.translation_ref()[2] = Tw2.translation[2]
        Tw3.translation_ref()[2] = Tw3.translation[2]

        if movecom:
            Tcom.translation_ref()[0] = a_x_com * t**3 + b_x_com * t**2 + c_x_com * t + d_x_com

        wheels[0].setPoseReference(Tw0)
        wheels[1].setPoseReference(Tw1)
        other_wheels[0].setPoseReference(Tw2)
        other_wheels[1].setPoseReference(Tw3)

        if movecom:
            com.setPoseReference(Tcom)
        
        t += dt
        time.sleep(dt)

    other_wheels[0].setBaseLink(u'car_frame')
    other_wheels[1].setBaseLink(u'car_frame')

    if movecom:
        com.disable()

    for item in to_be_disabled:
        item.enable()
        

# fct for spinning (i.e., rotating around the center of the sp -> rotation about the z-axis)
def spin(car, wheels, angle, duration, to_be_disabled, cli):

    for item in to_be_disabled:
        item.disable()

    Tcar = cli.getPoseFromTf('ci/car_frame', 'ci/world')

    Tw1 = cli.getPoseFromTf('ci/'+wheels[0].getDistalLink(), 'ci/car_frame')
    Tw2 = cli.getPoseFromTf('ci/'+wheels[1].getDistalLink(), 'ci/car_frame')
    Tw3 = cli.getPoseFromTf('ci/'+wheels[2].getDistalLink(), 'ci/car_frame')
    Tw4 = cli.getPoseFromTf('ci/'+wheels[3].getDistalLink(), 'ci/car_frame')
    
    R = Tcar.linear
    c_i = R[0, 0]
    s_i = R[1, 0]

    t = 0.0
    dt = 0.01

    angle_i = m.atan2(s_i, c_i)
    angle_f = angle_i + angle

    a, b, c, d = cubic(angle_i, angle_f, 0., 0., duration)

    while t < duration:
        angle = a * t**3 + b * t**2 + c * t + d

        c_t = m.cos(angle)
        s_t = m.sin(angle)

        R = np.array([[c_t, -s_t, 0.], [s_t, c_t, 0.], [0., 0., 1.]])

        for row in range(0, 3):
            for col in range(0, 3):
                Tcar.linear_ref()[row][col] = R[row][col]

        car.setPoseReference(Tcar)
        wheels[0].setPoseReference(Tw1)
        wheels[1].setPoseReference(Tw2)
        wheels[2].setPoseReference(Tw3)
        wheels[3].setPoseReference(Tw4)
        
        t += dt
        time.sleep(dt)

    for item in to_be_disabled:
        item.enable()


# fct for moving the car frame keeping the stance --- to be launched after the 4 steps (or 2) in order to bring the car frame at the center of the SP
def movecar(car, wheels, duration, to_be_disabled, cli):
    
    for item in to_be_disabled:
        item.disable()

    Tcar = cli.getPoseFromTf('ci/car_frame', 'ci/world')
    Tw1 = cli.getPoseFromTf('ci/wheel_1', 'ci/car_frame')
    Tw2 = cli.getPoseFromTf('ci/wheel_2', 'ci/car_frame')
    Tw3 = cli.getPoseFromTf('ci/wheel_3', 'ci/car_frame')
    Tw4 = cli.getPoseFromTf('ci/wheel_4', 'ci/car_frame')

    # initial position of car and wheels
    xcar_i = Tcar.translation[0]
    ycar_i = Tcar.translation[1]
    xw1_i = Tw1.translation[0]
    yw1_i = Tw1.translation[1]
    xw2_i = Tw2.translation[0]
    yw2_i = Tw2.translation[1]
    xw3_i = Tw3.translation[0]
    yw3_i = Tw3.translation[1]
    xw4_i = Tw4.translation[0]
    yw4_i = Tw4.translation[1]

    # final position of car (center of the SP)
    Tw1_world = np.matmul(Tcar.matrix(), Tw1.matrix())
    Tw2_world = np.matmul(Tcar.matrix(), Tw2.matrix())
    Tw3_world = np.matmul(Tcar.matrix(), Tw3.matrix())
    Tw4_world = np.matmul(Tcar.matrix(), Tw4.matrix())
    xcar_f = (Tw1_world[0, 3] + Tw2_world[0, 3] + Tw3_world[0, 3] + Tw4_world[0, 3])/4
    ycar_f = (Tw1_world[1, 3] + Tw2_world[1, 3] + Tw3_world[1, 3] + Tw4_world[1, 3])/4

    # final position of wheels wrt car
    xw1_f = 0.35
    yw1_f = 0.35
    xw2_f = 0.35
    yw2_f = -0.35
    xw3_f = -0.35
    yw3_f = 0.35
    xw4_f = -0.35
    yw4_f = -0.35
    
    a_carx, b_carx, c_carx, d_carx = cubic(xcar_i, xcar_f, 0., 0.,  duration)
    a_cary, b_cary, c_cary, d_cary = cubic(ycar_i, ycar_f, 0., 0., duration)
    a_w1x, b_w1x, c_w1x, d_w1x = cubic(xw1_i, xw1_f, 0., 0., duration)
    a_w1y, b_w1y, c_w1y, d_w1y = cubic(yw1_i, yw1_f, 0., 0., duration)
    a_w2x, b_w2x, c_w2x, d_w2x = cubic(xw2_i, xw2_f, 0., 0., duration)
    a_w2y, b_w2y, c_w2y, d_w2y = cubic(yw2_i, yw2_f, 0., 0., duration)
    a_w3x, b_w3x, c_w3x, d_w3x = cubic(xw3_i, xw3_f, 0., 0., duration)
    a_w3y, b_w3y, c_w3y, d_w3y = cubic(yw3_i, yw3_f, 0., 0., duration)
    a_w4x, b_w4x, c_w4x, d_w4x = cubic(xw4_i, xw4_f, 0., 0., duration)
    a_w4y, b_w4y, c_w4y, d_w4y = cubic(yw4_i, yw4_f, 0., 0., duration)
    
    t = 0.0
    dt = 0.01
    
    while t < duration:
        xcar_t = a_carx * t**3 + b_carx * t**2 + c_carx * t + d_carx
        ycar_t = a_cary * t**3 + b_cary * t**2 + c_cary * t + d_cary
        xw1_t = a_w1x * t**3 + b_w1x * t**2 + c_w1x * t + d_w1x
        yw1_t = a_w1y * t**3 + b_w1y * t**2 + c_w1y * t + d_w1y
        xw2_t = a_w2x * t**3 + b_w2x * t**2 + c_w2x * t + d_w2x
        yw2_t = a_w2y * t**3 + b_w2y * t**2 + c_w2y * t + d_w2y
        xw3_t = a_w3x * t**3 + b_w3x * t**2 + c_w3x * t + d_w3x
        yw3_t = a_w3y * t**3 + b_w3y * t**2 + c_w3y * t + d_w3y
        xw4_t = a_w4x * t**3 + b_w4x * t**2 + c_w4x * t + d_w4x
        yw4_t = a_w4y * t**3 + b_w4y * t**2 + c_w4y * t + d_w4y

        Tcar.translation_ref()[0] = xcar_t
        Tcar.translation_ref()[1] = ycar_t
        Tw1.translation_ref()[0] = xw1_t
        Tw1.translation_ref()[1] = yw1_t
        Tw2.translation_ref()[0] = xw2_t
        Tw2.translation_ref()[1] = yw2_t
        Tw3.translation_ref()[0] = xw3_t
        Tw3.translation_ref()[1] = yw3_t
        Tw4.translation_ref()[0] = xw4_t
        Tw4.translation_ref()[1] = yw4_t
        
        car.setPoseReference(Tcar)
        wheels[0].setPoseReference(Tw1)
        wheels[1].setPoseReference(Tw2)
        wheels[2].setPoseReference(Tw3)
        wheels[3].setPoseReference(Tw4)
        
        t += dt
        time.sleep(2*dt)

    for item in to_be_disabled:
        item.enable()
        

# fct for stepping. the step is a semi-circumference. every 0.1s, it is checked if the com lies in the support triangle
def step(moving_foot, still_feet, step_length, duration, to_be_disabled, car, com, cli, filename_pos, filename_vel):

    for item in to_be_disabled:
        item.disable()

    com.enable()   
    
    com_pos = np.loadtxt('/home/matteo/catkin_ws/src/centauro_cartesio-devel-cpack/python/' + filename_pos)
    com_vel = np.loadtxt('/home/matteo/catkin_ws/src/centauro_cartesio-devel-cpack/python/' + filename_vel)
    interval_duration = 0.1

    t = 0.00
    dt = 0.01
    period = 4.0
    radius = step_length/2

    T1 = 1.0
    T2 = 3.0

    moving_foot.setBaseLink(u'world')
    for foot in still_feet:
        foot.setBaseLink(u'world')

    Tmoving_foot = (cli.getPoseFromTf('ci/'+moving_foot.getDistalLink(), 'ci/world'))
    Tmoving_init = Tmoving_foot.translation
    Tw1 = cli.getPoseFromTf('ci/'+still_feet[0].getDistalLink(), 'ci/world')
    Tw2 = cli.getPoseFromTf('ci/'+still_feet[1].getDistalLink(), 'ci/world')
    Tw3 = cli.getPoseFromTf('ci/'+still_feet[2].getDistalLink(), 'ci/world')

    Tcar = cli.getPoseFromTf('ci/car_frame', 'ci/world')
    Tcom = cli.getPoseFromTf('ci/com', 'ci/world')

    counter = 0
    i = 0

    total = int(duration*100)
    '''
    # first "control" loop that brings the com from the current position to the one imposed by casadi
    a_x_com, b_x_com, c_x_com, d_x_com = cubic(Tcom.translation[0], data[0][0], 3.0)
    a_y_com, b_y_com, c_y_com, d_y_com = cubic(Tcom.translation[1], data[0][1], 3.0)
    a_z_com, b_z_com, c_z_com, d_z_com = cubic(Tcom.translation[2], data[0][2], 3.0)
    while t < 3.0:
       Tcom.translation_ref()[0] = a_x_com * t**3 + b_x_com * t**2 + c_x_com * t + d_x_com
       Tcom.translation_ref()[1] = a_y_com * t**3 + b_y_com * t**2 + c_y_com * t + d_y_com
       Tcom.translation_ref()[2] = a_z_com * t**3 + b_z_com * t**2 + c_z_com * t + d_z_com
       com.setPoseReference(Tcom)
       t += dt
       time.sleep(dt)
    t = 0.0
    '''
    while counter < total:
       
       if counter%10 == 0:
           com_init = com_pos[i]
           com_goal = com_pos[i+1]
           dcom_init = com_vel[i]
           dcom_goal = com_vel[i+1]
           
           a_comx, b_comx, c_comx, d_comx = cubic(com_init[0], com_goal[0], dcom_init[0], dcom_goal[0], interval_duration)
           a_comy, b_comy, c_comy, d_comy = cubic(com_init[1], com_goal[1], dcom_init[1], dcom_goal[1], interval_duration)
           a_comz, b_comz, c_comz, d_comz = cubic(com_init[2], com_goal[2], dcom_init[2], dcom_goal[2], interval_duration)
           i += 1

           print 'com_init: ' + str(com_init)
           print 'com_goal: ' + str(com_goal)
           print 'com: ' + str(Tcom.translation)

       Tcom.translation_ref()[0] = a_comx * (t-(i-1)*interval_duration)**3 + b_comx * (t-(i-1)*interval_duration)**2 + c_comx * (t-(i-1)*interval_duration) + d_comx
       Tcom.translation_ref()[1] = a_comy * (t-(i-1)*interval_duration)**3 + b_comy * (t-(i-1)*interval_duration)**2 + c_comy * (t-(i-1)*interval_duration) + d_comy
       Tcom.translation_ref()[2] = a_comz * (t-(i-1)*interval_duration)**3 + b_comz * (t-(i-1)*interval_duration)**2 + c_comz * (t-(i-1)*interval_duration) + d_comz + 0.3
       
       Tw1.translation_ref()[0] = Tw1.translation[0]
       Tw1.translation_ref()[1] = Tw1.translation[1]
       Tw1.translation_ref()[2] = Tw1.translation[2]
       Tw2.translation_ref()[0] = Tw2.translation[0]
       Tw2.translation_ref()[1] = Tw2.translation[1]
       Tw2.translation_ref()[2] = Tw2.translation[2]
       Tw3.translation_ref()[0] = Tw3.translation[0]
       Tw3.translation_ref()[1] = Tw3.translation[1]
       Tw3.translation_ref()[2] = Tw3.translation[2]
       
       if t >= T1 and t <= T2:
           delta_x = radius - radius * m.cos((t-T1)*m.pi/(T2-T1))
           delta_z = radius * m.sin((t-T1)*m.pi/(T2-T1))
           Tmoving_foot.translation_ref()[0] = delta_x + Tmoving_init[0]
           Tmoving_foot.translation_ref()[1] = Tmoving_init[1]
           Tmoving_foot.translation_ref()[2] = delta_z + Tmoving_init[2]
       else:  
           Tmoving_foot.translation_ref()[0] = Tmoving_foot.translation[0]
           Tmoving_foot.translation_ref()[1] = Tmoving_foot.translation[1]
           Tmoving_foot.translation_ref()[2] = Tmoving_foot.translation[2]

       com.setPoseReference(Tcom)
       still_feet[0].setPoseReference(Tw1)
       still_feet[1].setPoseReference(Tw2)
       still_feet[2].setPoseReference(Tw3)
       moving_foot.setPoseReference(Tmoving_foot)
       counter += 1
       
       t += dt
       time.sleep(5*dt)

    com.disable()
    for item in to_be_disabled:
        item.enable()

    moving_foot.setBaseLink(u'car_frame')
    for foot in still_feet:
        foot.setBaseLink(u'car_frame')


#########################################################
########################## MAIN #########################
#########################################################

def main():

    start_time = time.time()
    
    k = 0
    
    # load file with primitives and number of times they are applied
    plan = np.loadtxt('/home/matteo/catkin_ws/src/centauro_cartesio-devel-cpack/python/plan.txt')
    primitives = plan[:, 0]
    times = plan[:, 1]

    cli = pyci.CartesianInterfaceRos()      # initialization of cartesIO

    # tasks
    com = cli.getTask('Com')
    car = cli.getTask('car_frame')
    pelvis = cli.getTask('pelvis')
    w1 = cli.getTask('wheel_1')
    w2 = cli.getTask('wheel_2')
    w3 = cli.getTask('wheel_3')
    w4 = cli.getTask('wheel_4')
    rw1 = cli.getTask('rolling_wheel_1')
    rw2 = cli.getTask('rolling_wheel_2')
    rw3 = cli.getTask('rolling_wheel_3')
    rw4 = cli.getTask('rolling_wheel_4')

    #homing(car, w1, w2, w3, w4, pelvis, com)
    
    n_primitives = len(primitives)

    # executes planner indications
    for i in range(0, n_primitives):
                
        primitive = primitives[i]
        application = times[i]
      
        if primitive == 0:
            print(color.BOLD + 'Primitive 0: clockwise spin of 10 deg for ' + str(int(application)) + ' times. (' + str(i+1) + '/' + str(n_primitives) + ')' + color.END)
            spin(car, [w1, w2, w3, w4], -m.pi/18 * application, 5.0 * application, [com], cli)
        elif primitive == 1:
            print(color.BOLD + 'Primitive 1: counter-clockwise spin of 10 deg for ' + str(int(application)) + ' times. (' + str(i+1) + '/' + str(n_primitives) + ')' + color.END)
            spin(car, [w1, w2, w3, w4], m.pi/18 * application, 5.0 * application, [com], cli)
        elif primitive == 2:
            print(color.BOLD + 'Primitive 2: forward roll of 0.05 m for ' + str(int(application)) + ' times. (' + str(i+1) + '/' + str(n_primitives) + ')' + color.END)
            roll(car, [w1, w2, w3, w4], 0.05 * application, 'x', 1.0 * application, [com], cli)
        elif primitive == 3:
            print(color.BOLD + 'Primitive 3: backward roll of 0.05 m for ' + str(int(application)) + ' times. (' + str(i+1) + '/' + str(n_primitives) + ')' + color.END)
            roll(car, [w1, w2, w3, w4], -0.05 * application, 'x', 1.0 * application, [com], cli)
        elif primitive == 4:
            print(color.BOLD + 'Primitive 4: right roll of 0.05 m for ' + str(int(application)) + ' times. (' + str(i+1) + '/' + str(n_primitives) + ')' + color.END)
            roll(car, [w1, w2, w3, w4], 0.05 * application, 'y', 1.0 * application, [com], cli)
        elif primitive == 5:
            print(color.BOLD + 'Primitive 5: left roll of 0.05 m for ' + str(int(application)) + ' times. (' + str(i+1) + '/' + str(n_primitives) + ')' + color.END)
            roll(car, [w1, w2, w3, w4], -0.05 * application, 'y', 1.0 * application, [com], cli)
        else:
           
            if k == 0:
               
                print(color.BOLD + 'Preparation to step: forward roll 0.20 m with back wheels.' + color.END)
                rollTwoWheelsandMoveCom([w3, w4], 0.2, com, 0., False, [w1, w2], 4.0, [], cli)
                #time.sleep(3.0)
                cli.update()
                #return 0
            
            k += 1
            if primitive == 6:
                print(color.BOLD + 'Primitive 6: step of 0.20 m with BR foot. (' + str(i+1) + '/' + str(n_primitives) + ')' + color.END)
                step(moving_foot = w4, still_feet = [w1, w2, w3], step_length = 0.2, duration = 4.0, to_be_disabled = [pelvis], car=car, com=com, cli=cli, filename_pos='com_traj_with_com_vel02/COMtraj_BR.txt', filename_vel = 'com_traj_with_com_vel02/DCOMtraj_BR.txt')
            elif primitive == 7:
                print(color.BOLD + 'Primitive 7: step of 0.20 m with BL foot. (' + str(i+1) + '/' + str(n_primitives) + ')' + color.END)
                step(moving_foot = w3, still_feet = [w1, w2, w4], step_length = 0.2, duration = 4.0, to_be_disabled = [pelvis], car=car, com=com, cli=cli, filename_pos='com_traj_with_com_vel02/COMtraj_BL.txt', filename_vel = 'com_traj_with_com_vel02/DCOMtraj_BL.txt')
            elif primitive == 8:
                print(color.BOLD + 'Primitive 8: step of 0.20 m with FR foot. (' + str(i+1) + '/' + str(n_primitives) + ')' + color.END)
                cli.update()
                step(moving_foot = w2, still_feet = [w1, w3, w4], step_length = 0.2, duration = 4.0, to_be_disabled = [pelvis], car=car, com=com, cli=cli, filename_pos='com_traj_with_com_vel02/COMtraj_FR.txt', filename_vel = 'com_traj_with_com_vel02/DCOMtraj_FR.txt')
            elif primitive == 9:
                print(color.BOLD + 'Primitive 9: step of 0.20 m with FL foot. (' + str(i+1) + '/' + str(n_primitives) + ')' + color.END)
                step(moving_foot = w1, still_feet = [w2, w3, w4], step_length = 0.2, duration = 4.0, to_be_disabled = [pelvis], car=car, com=com, cli=cli, filename_pos='com_traj_with_com_vel02/COMtraj_FL.txt', filename_vel = 'com_traj_with_com_vel02/DCOMtraj_FL.txt')
            
        #time.sleep(3.0)
        cli.update()
        
        if k == 4:
            print(color.BOLD + 'Conclusion of step: forward roll 0.20 m with front wheels.' + color.END)
            rollTwoWheelsandMoveCom([w1, w2], 0.2, com, 0.1, True, [w3, w4], 3.0, [pelvis], cli)
            #time.sleep(3.0)
            cli.update()

            print(color.BOLD + 'Realigning car_frame with the center of the support polygon...' + color.END)
            movecar(car, [w1, w2, w3, w4], 5.0, [pelvis], cli)
            #time.sleep(3.0)
            cli.update()
            
            k = 0
    
    print(color.BOLD + color.GREEN + 'Execution completed in ' + str(time.time() - start_time) + ' s. \n' + color.END)
    
# main
if __name__ == "__main__":
    main()
    
