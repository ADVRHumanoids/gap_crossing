#!/usr/bin/env python  
import os
import numpy as np

plan = np.loadtxt('/home/matteo/catkin_ws/src/centauro_cartesio-devel-cpack/python/try_roll.txt')
primitives = plan[:, 0]
times = plan[:, 1]

n_primitives = len(primitives)
for i in range(1, n_primitives):
   out_file = open('/home/matteo/catkin_ws/src/centauro_cartesio-devel-cpack/python/temp.txt', 'w')
   out_file.write(str(i) + '\n')
   out_file.close()
   execfile('/home/matteo/catkin_ws/src/centauro_cartesio-devel-cpack/python/simple_commands_callable.py')
   os.remove('/home/matteo/catkin_ws/src/centauro_cartesio-devel-cpack/python/temp.txt')
   
