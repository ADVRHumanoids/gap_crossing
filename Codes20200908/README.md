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
Using the NLP-based method, you can plan a sequence of static poses (stance and CoMs location) for a quadrupedal robot running `Planning/04. Quadruped_CASADI/quadruped_planner.py`.

Variables that can be decided by user:
- number of static stances `ns` (line 10);
- masses of virtual bipeds `mB`, `mF` (lines 20, 21);
- bounds and initial guess on CoMs, contact positions and contact forces (lines 27-40);
- final CoM locations (lines 42, 43);
- gap location (lines 46-50);
- maximum distance that can be traveled by a foot during a movement (line 53);
- (fixed) leg length (line 55);
- nominal length and width of the support polygon and their allowed variations (lines 57-60);
- friction cones specifications (lines 120-121);
- solver options (lines 343-345).

### Planning with OMPL
The following information are referred to the code in `Planning/03. Quadruped_as_Unicycle_OMPL/B. With_Gap/src/Quadruped_Unicycle.cpp`. Using the sampling-based method, you can plan a sequence of stances for a quadrupedal robot.

Variables that can be decided by user:
- macros (lines 23-49) regarding solver parameters, dimension of the movements, environment specifications (step activation zone, gap, obstacle), robot geometry (length and width);
- environmental bounds (lines 506-509);
- control duration (line 528);
- start configuration (lines 540-543);
- goal configuration (lines 558-561).

Once a path is returned (`path.txt`), it can be seen using `Planning/03. Quadruped_as_Unicycle_OMPL/B. With_Gap/plot.py`, changing the parameters of the environment and the start and goal configuration if needed (lines 38-67). Additionally, finishing operations (aggregation of primitives of "the same kind") can be performed running `Planning/03. Quadruped_as_Unicycle_OMPL/B. With_Gap/clean.py`.

Similarly for the other sampling-based planners (`Planning/01. Compass_OMPL`, `Planning/02. Biped_OMPL`, `Planning/03. Quadruped_as_Unicycle_OMPL/A. No_Step`).

### CoM Trajectory Generation
After generating a path with OMPL, optimal CoM trajectory for the step tasks can be generated running the code in `Tasks/Com_Traj_for_Steps_CASADI/Com_Traj.py`. 

Variables that can be decided by user:
- mass of the robot (line 7);
- step length (line 15, compliant with the planner);
- timing specifications (lines 20-24);
- bounds (lines 64-72);
- weights of cost function (lines 98-102);
- friction cones specifications (lines 129-130);
- solver options (lines 406-408).

### Whole-body Motion Generation
In the `centauro_cartesio-devel-cpack` package in your catkin workspace, in the `Python` folder, place subfolders containing the file with the path and a folder with the CoM trajectories.

If you want, you can modify `centauro_car_model_stack.yaml` in order to change the hierarchy of tasks in the `configs` folder (not recommended).

Cartesian trajectories are designed in the file `Cartesian_Task.py`, found in the `Python` folder. Variables that can be decided by user:
- sampling time for the Cartesian trajectories (lines 67, 130, 184);
- name of the simulation (that is, the name of the folder containing the files) (line 434);
- Python folder path (line 435);
- primitive dimensions (lines 461-463, compliant with the planner);
- primitive timings (lines 466-468, compliant with the CoM trajectory generation for the step along with lines 325-326 for the step phases).

To run CartesI/O for the whole-body motion generation:
1. Launch `roscore`;
2. Run `mon launch centauro_cartesio centauro_car_model.launch`;
3. Launch `rviz` (only for visualization purposes);
4. Run `rosrun centauro_cartesio Cartesian_Task.py` in order to execute the path and the relative Cartesian trajectories.
