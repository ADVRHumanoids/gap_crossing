# Code
Folder containing all the code written for planning and whole-body motion generation for CENTAURO. It is assumed that the robot moves on a flat horizontal terrain, in presence of a gap and a (fixed) obstacle.

## Install
Code completely developed on Ubuntu 16.04 partition. 

Required tools:
- Open Motion Planning Library (OMPL) v. 1.4.2 (or later): [download](https://ompl.kavrakilab.org/download.html). Standalone library for sampling-based motion planning.
- CasADi v. 3.4.5 (or later): [download](https://web.casadi.org/get/). Open-source tool for nonlinear optimization and algorithmic differentiation.
- ROS Kinetic Kame: [download](http://wiki.ros.org/kinetic/Installation). 
- pip v. 20.0.2 (or later).
- Python v. 2.7 (or later).
- cmake v. 3.5.1 (or later).

To install:
1. Download the whole folder.
2. Unpack `xenial-16.04-2020_09_09_12_54_56.zip` found in the `Debian` folder.
3. Install the debian running `install.sh`.
4. Unpack all the packages found in the `Tasks/ROS_Packages` folder in your catkin workspace.
5. Build the catkin workspace.

## Run
### Planning with CasADi
Using the NLP-based method, you can plan a sequence of static poses (stance and CoMs location) for a quadruped robot running `Planning/04. Quadruped_CASADI/quadruped_planner.py`.

Variables that can be decided by user:
- number of static stances `ns` (line 10);
- masses of virtual bipeds `mB`, `mF` (lines 20, 21);
- bounds and initial guess on CoMs, contact positions and contact forces (lines 27-40);
- final CoM locations (lines 42, 43);
- gap location (lines 46-50);
- maximum distance that can be traveled by a foot during a movement (line 53);
- friction cones specifications (lines 102-103);
