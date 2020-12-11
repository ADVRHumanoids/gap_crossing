#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <math.h>       

#include <ompl/geometric/PathGeometric.h>

#include <iostream>
#include <fstream>

#define MAX_ITERATION 	10
#define TIME		  	5.0	

#define PI 3.14159265
#define L 0.3
#define DELTA_ROT PI/6
#define GAP_LENGTH 0.1

#define X_MAX 2.5
#define X_MIN -2.5
#define Y_MAX 2.5
#define Y_MIN -2.5

#define OBS_X_MAX -0.5
#define OBS_X_MIN -1.0
#define OBS_Y_MAX 1.0
#define OBS_Y_MIN -1.0

#define X_GAP 0.75

namespace ob = ompl::base;
namespace oc = ompl::control;

// At each iteration:
// 1) The planner sample a state in order to expand the tree;
// 2) It finds the nearest available vertex from previous iterations
// 3) Try to connect the two vertex using a control action
// 4) Continues until the goal region is reached


double mod2PI(double angle)
{
	if (angle < -2*PI)
	{
		angle += 2*PI;
	}
	else
	{
		if (angle > 2*PI)
		{
			angle -= 2*PI;
		}
	}
	return angle;
}

// A control-based motion planning problem requires a propagate function that associates a control action to a state variation. 
// Specifically, DiscreteControlSpace pick a random int between the lower and upper bounds defined in the constructor and propagates the state
// depending on this value (see switch-case in propagate function)
// params: start is the state that must be propagated;
//         result is the propagated state;
//         duration is the amount of time during which the control is applied (not used in this case)
void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    // Cast the start state to the desired type (consistent to the state space defined in plan())
    auto se2state = start->as<ompl::base::SE2StateSpace::StateType>();
    
    // Extract x,y position of the right (R) foot and orientation of the body
    auto posR = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
    auto rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1)->value;
    
    // Cast the control to the desired type (consistent to the control space defined in plan())
    auto ctrl = control->as<oc::DiscreteControlSpace::ControlType>()->value;

    // Cast the result state to the desired type (consistent to the state space defined in plan())
    auto r_result = result->as<ob::SE2StateSpace::StateType>();
    
    double xR, yR, theta;
    
    const double posL[2] = {posR[0] - cos(rot)*L, posR[1] - sin(rot)*L};

    
    // Pick a control and propagate the state (x and y will be only propagated)
    switch (ctrl)
    {
        case 0:
			// rotate the body of 30 deg keeping the right foot fixed
			theta = mod2PI(rot+DELTA_ROT);
            r_result->setXY(posR[0], posR[1]);
            r_result->setYaw(theta);
            break;
        case 1:
			// rotate the body of -30 deg keeping the right foot fixed
			theta = mod2PI(rot-DELTA_ROT);
            r_result->setXY(posR[0], posR[1]);
            r_result->setYaw(theta);
            break;
        case 2:
			// rotate the body of 30 deg keeping the left foot fixed
            theta = mod2PI(rot+DELTA_ROT);
            xR = posL[0] + cos(theta)*L;
            yR = posL[1] + sin(theta)*L;
            r_result->setXY(xR, yR);
            r_result->setYaw(theta);
            break;
        case 3:
            // rotate the body of 30 deg keeping the left foot fixed
            theta = mod2PI(rot-DELTA_ROT);
            xR = posL[0] + cos(theta)*L;
            yR = posL[1] + sin(theta)*L;
            r_result->setXY(xR, yR);
            r_result->setYaw(theta);
            break;
    }        
}

// State validity function: returns true if the state sampled satisfies the bounds and if it is not inside a gap)
bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    // Cast the state to the desired value and extract the position
    auto se2state = state->as<ob::SE2StateSpace::StateType>();
    const auto rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1)->value;
    const auto posR = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double posL[2] = {posR[0] - cos(rot)*L, posR[1] - sin(rot)*L};
    
	// check left foot satisfy bounds
	bool bound_checkL = X_MIN < posL[0] && posL[0] < X_MAX && Y_MIN < posL[1] && posL[1] < Y_MAX;
	
    // Check for state validity
    bool obstacle_checkR = !(OBS_X_MIN < posR[0] && posR[0] < OBS_X_MAX && OBS_Y_MIN < posR[1] && posR[1] < OBS_Y_MAX);
    bool obstacle_checkL = !(OBS_X_MIN < posL[0] && posL[0] < OBS_X_MAX && OBS_Y_MIN < posL[1] && posL[1] < OBS_Y_MAX);
    // gap crossing
    bool gap_checkR = posR[0] < X_GAP-GAP_LENGTH/2 || posR[0] > X_GAP+GAP_LENGTH/2;
    bool gap_checkL = posL[0] < X_GAP-GAP_LENGTH/2 || posL[0] > X_GAP+GAP_LENGTH/2;
    
    return si->satisfiesBounds(state) && bound_checkL && obstacle_checkR && obstacle_checkL && gap_checkR && gap_checkL;
}

void plan()
{
	ob::PlannerStatus solved = ob::PlannerStatus::UNKNOWN;
	int n = 0;
	do {
		// Create the state space
		auto space = std::make_shared<ompl::base::SE2StateSpace>();
		
		// Create bounds and set them
		ob::RealVectorBounds bounds(2);
		bounds.setLow(0, X_MIN);
		bounds.setLow(1, Y_MIN);
		bounds.setHigh(0, X_MAX);
		bounds.setHigh(1, Y_MAX);
		space->setBounds(bounds);
		
		// Create the control space
		auto cspace = std::make_shared<ompl::control::DiscreteControlSpace>(space, 0, 3);
		
		// Create an instance to the SpaceInformation
		auto si(std::make_shared<oc::SpaceInformation>(space, cspace));
		
		// In the propagation of the tree, a primitive can be applied only one at time 
			si-> setMinMaxControlDuration(1, 1);
			si->setPropagationStepSize(1.0);

		// Set the state validity checker function
		si->setStateValidityChecker(
			[&si](const ob::State *state) { return isStateValid(si.get(), state); });
	 
		// Set the propagator
		si->setStatePropagator(propagate);
	 
		// Create a start state
		ob::ScopedState<ob::SE2StateSpace> start(space);
		start->setX(-2.0);
		start->setY(-0.15);
		start->setYaw(-PI/2);
	 
		// Create a goal state
		ob::ScopedState<ob::SE2StateSpace> goal(space);
		goal->setX(2.0);
		goal->setY(-0.15);
		goal->setYaw(-PI/2);
		 
		// Create a problem instance
		auto pdef(std::make_shared<ob::ProblemDefinition>(si));
	 
		// Cet the start and goal states
		pdef->setStartAndGoalStates(start, goal, 0.05);
	 
		// Create a planner for the defined space
		auto planner(std::make_shared<oc::RRT>(si));
		//auto planner(std::make_shared<oc::EST>(si));
		//auto planner(std::make_shared<oc::KPIECE1>(si));
		
		// Create the planner starting from the information collected before
		planner->setProblemDefinition(pdef);
		planner->setup();
	 
		// Print the settings for this space (not mandatory)
		si->printSettings(std::cout);
	 
		// Print the problem settings (not mandatory)
		pdef->print(std::cout);
	 
		n++; 
		std::cout << "----------------------------------------------------------------------------"<< std::endl;
		std::cout << "\nStarting iteration number " << n << "\n"<< std::endl;
		
		solved = planner->ob::Planner::solve(TIME * n);
		
		std::cout << "\nOutcome: " << solved << "\n"<< std::endl;
		
		if (solved == ob::PlannerStatus::EXACT_SOLUTION || n == MAX_ITERATION)
		//if (solved)
		{
			ob::PathPtr path = pdef->getSolutionPath();
			
			path->print(std::cout);
			
			auto numC = path->as<oc::PathControl>()->getControlCount();
			for (int i = 0; i < numC; i++)
			{
				cspace->as<oc::ControlSpace>()->printControl(path->as<oc::PathControl>()->getControl(i), std::cout);
			}
			
			std::cout << "\n" << solved << " found in " << n << " iteration(s)\n"<< std::endl;
			
			std::ofstream log_file;
			log_file.open ("log.txt");
			path->print(log_file);
			log_file.close();
        
        
			std::ofstream path_file;
			path_file.open ("path.txt");
			// path->as<ompl::geometric::PathGeometric>()->printAsMatrix(path_file);
			path->as<ompl::control::PathControl>()->printAsMatrix(path_file);
			path_file.close();
			
		} 
	
	}
    while(solved != ob::PlannerStatus::EXACT_SOLUTION && n < MAX_ITERATION);     
}

int main()
{
    plan();
    return 0;
}
