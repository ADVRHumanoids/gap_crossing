#include <vector>
#include <iostream>
#include <ompl/base/StateSpace.h>
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
#include <stdio.h>      
#include <iostream>
#include <fstream>

//vedi SST

#define MAX_ITERATION 1

#define PI 3.14159265
#define L 1.0

//Movement lenghts
#define DELTA_ROT 		PI/6
#define GAP_LENGTH 		L/10
#define SHORT_STEP 		0.25
#define LONG_STEP 		0.5
#define LATERAL_STEP 	0.1
#define TOL 			0.001
#define SHORT_STEP_DIST sqrt(pow(L, 2) + pow(SHORT_STEP, 2))
#define LONG_STEP_DIST 	sqrt(pow(L, 2) + pow(LONG_STEP, 2))

//Rotations
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
    auto R2_R2_SO2state = start->as<ob::CompoundStateSpace::StateType>();
    
    // Extract x,y position of the right (R) and left foot(L) and orientation of the body
    auto posR = R2_R2_SO2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    double xR = posR[0], yR = posR[1];
    auto posL = R2_R2_SO2state->as<ob::RealVectorStateSpace::StateType>(1)->values;
    double xL = posL[0], yL = posL[1];
    auto theta = R2_R2_SO2state->as<ob::SO2StateSpace::StateType>(2)->value;
    double feet_dist = sqrt(pow(xR-xL,2) + pow(yR-yL,2));
    
    // Cast the control to the desired type (consistent to the control space defined in plan())
    auto ctrl = control->as<oc::DiscreteControlSpace::ControlType>()->value;

	// Cast the result state to the desired type (consistent to the state space defined in plan())
    auto r_result = result->as<ob::CompoundStateSpace::StateType>();
  
	// Pick a control and propagate the state (x and y will be only propagated)
	double xR_new = xR, yR_new = yR, xL_new = xL, yL_new = yL, theta_new = theta;
    switch (ctrl)
    {
		case(CW_AROUND_R):
			if(feet_dist < L+TOL)
			{
				 theta_new = mod2PI(theta+DELTA_ROT);
				 xL_new = xR_new + feet_dist * cos(theta_new);
				 yL_new = yR_new + feet_dist * sin(theta_new);
			} 
			break;
		case(CCW_AROUND_R):
			if(feet_dist < L+TOL)
			{
				 theta_new = mod2PI(theta-DELTA_ROT);
				 xL_new = xR_new + feet_dist * cos(theta_new);
				 yL_new = yR_new + feet_dist * sin(theta_new);
			} 
			break;
		case(CW_AROUND_L):
			if(feet_dist < L+TOL)
			{
				 theta_new = mod2PI(theta+DELTA_ROT);
				 xR_new = xL_new - feet_dist * cos(theta_new);
				 yR_new = yL_new - feet_dist * sin(theta_new);				
			} 
			break;
		case(CCW_AROUND_L):
			if(feet_dist < L+TOL)
			{
				 theta_new = mod2PI(theta-DELTA_ROT);
				 xR_new = xL_new - feet_dist * cos(theta_new);
				 yR_new = yL_new - feet_dist * sin(theta_new);
			} 
			break;	
			
		case(SH_FW_R):
			//std::cout << SH_FW_R << std::endl;
			xR_new += SHORT_STEP * sin(theta_new);
			yR_new -= SHORT_STEP * cos(theta_new);
			break;
		
		case(SH_BW_R):
			//std::cout << SH_BW_R << std::endl;
			xR_new -= SHORT_STEP * sin(theta_new);
			yR_new += SHORT_STEP * cos(theta_new);
			break;
			
		case(SH_FW_L):
			//std::cout << SH_FW_L << std::endl;
			xL_new += SHORT_STEP * sin(theta_new);
			yL_new -= SHORT_STEP * cos(theta_new);
			break;
			
		case(SH_BW_L):
			//std::cout << SH_BW_L << std::endl;
			xL_new -= SHORT_STEP * sin(theta_new);
			yL_new += SHORT_STEP * cos(theta_new);
			break;
			
		case(LO_FW_R):
			//std::cout << LO_FW_R << std::endl;
			xR_new += LONG_STEP * sin(theta_new);
			yR_new -= LONG_STEP * cos(theta_new);
			break;
			
		case(LO_FW_L):
			//std::cout << LO_FW_L << std::endl;
			xL_new += LONG_STEP * sin(theta_new);
			yL_new -= LONG_STEP * cos(theta_new);
			break;
			
		case(R_LAT_R):
			if(feet_dist < SHORT_STEP_DIST + TOL)
			{
				xR_new -= LATERAL_STEP * cos(theta_new);
				yR_new -= LATERAL_STEP * sin(theta_new);
			}
			break;
			
		case(L_LAT_R):
			xR_new += LATERAL_STEP * cos(theta_new);
			yR_new += LATERAL_STEP * sin(theta_new);
			break;
			
		case(R_LAT_L):
			xL_new -= LATERAL_STEP * cos(theta_new);
			yL_new -= LATERAL_STEP * sin(theta_new);
			break;
			
		case(L_LAT_L):
			if(feet_dist < SHORT_STEP_DIST + TOL)
			{
				xL_new += LATERAL_STEP * cos(theta_new);
				yL_new += LATERAL_STEP * sin(theta_new);
			}
			break;	
	}
	
	r_result->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = xR_new;
	r_result->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = yR_new;
	r_result->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = xL_new;
	r_result->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = yL_new;
	r_result->as<ob::SO2StateSpace::StateType>(2)->value = theta_new;
}





// State validity function: returns true if the state sampled satisfies the bounds and if it is not inside a gap)
bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{   
	// Cast the start state to the desired type (consistent to the state space defined in plan())
    auto R2_R2_SO2state = state->as<ob::CompoundStateSpace::StateType>();
    
    // Extract x,y position of the right (R) and left foot(L) and orientation of the body
    auto posR = R2_R2_SO2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    auto posL = R2_R2_SO2state->as<ob::RealVectorStateSpace::StateType>(1)->values;
    auto theta = R2_R2_SO2state->as<ob::SO2StateSpace::StateType>(2)->value;
    
    double feet_dist = sqrt(pow(posR[0]-posL[0],2) + pow(posR[1]-posL[1],2));
	
	bool check_bounds = si->satisfiesBounds(state);
	bool check_dist_min = feet_dist > L-TOL;
	bool check_dist_max = feet_dist < LONG_STEP_DIST+TOL;
	
	bool check_gapR = posR[0] < -GAP_LENGTH/2 || posR[0] > GAP_LENGTH/2;
	bool check_gapL = posL[0] < -GAP_LENGTH/2 || posL[0] > GAP_LENGTH/2;
	
	bool check_obsR = posR[0] < 1 || posR[0] > 1.5 || posR[1] < -0.5 || posR[1] > 0.5;
	bool check_obsL = posL[0] < 1 || posL[0] > 1.5 || posL[1] < -0.5 || posL[1] > 0.5;
	
	//std::cout << "----" << std::endl;
	//std::cout << "check_bounds: " << check_bounds << std::endl;
	//std::cout << "check_dist_min: " << check_dist_min << std::endl;
	//std::cout << "check_dist_max: " << check_dist_max << std::endl;
	
    return check_bounds && check_dist_min && check_dist_max && check_gapR && check_gapL && check_obsR && check_obsL;
}





void plan()
{
	ob::PlannerStatus solved = ob::PlannerStatus::UNKNOWN;
	int n = 0;
	do {
		n++;
		std::cout << "----------------------------------------------------------------------------"<< std::endl;
		std::cout << "\n\nStarting itereation number " << n << "\n\n"<< std::endl;
		// Create the state space
		auto R2R = std::make_shared<ob::RealVectorStateSpace>(2);
		auto R2L = std::make_shared<ob::RealVectorStateSpace>(2);
		
		ob::RealVectorBounds bounds(2);
		bounds.setLow(0, -20);
		bounds.setLow(1, -20);
		bounds.setHigh(0, 20);
		bounds.setHigh(1, 20);
		R2R->setBounds(bounds);
		R2L->setBounds(bounds);
			
		auto SO2 = std::make_shared<ob::SO2StateSpace>();
		std::vector<ob::StateSpacePtr> components {R2R, R2L, SO2};
		std::vector<double> weights {1.0, 1.0, 1.0};
		auto space = std::make_shared<ob::CompoundStateSpace>(components, weights);
		
		// Create the control space
		auto cspace = std::make_shared<ompl::control::DiscreteControlSpace>(space, 0, 13);
		
		// Create an instance to the SpaceInformation
		auto si(std::make_shared<oc::SpaceInformation>(space, cspace));
		
		// In the propagation of the tree, a primitive can be applied only one at time 
		si-> setMinMaxControlDuration(1, 1);
		//si->setPropagationStepSize(1.16279);
		// Set the state validity checker function
		si->setStateValidityChecker(
			[&si](const ob::State *state) { return isStateValid(si.get(), state); });
	 
		// Set the propagator
		si->setStatePropagator(propagate);

		// Create a start state
		
		ob::ScopedState<ob::CompoundStateSpace> start(space);
		
		double xR_start = -1.0, yR_start = -0.5, xL_start = -1.0, yL_start = 0.5;
		double theta_start = atan2(yL_start-yR_start, xL_start-xR_start);
		
		start->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = xR_start;
		start->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = yR_start;
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = xL_start;
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = yL_start;
		start->as<ob::SO2StateSpace::StateType>(2)->value = theta_start;
		

		// Create a goal state
		ob::ScopedState<ob::CompoundStateSpace> goal(space);
		
		double xR_goal = 2.0, yR_goal = 0.0, xL_goal = 3.0, yL_goal = 0.0;
		double theta_goal = atan2(yL_goal-yR_goal, xL_goal-xR_goal);
		
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = xR_goal;
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = yR_goal;
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = xL_goal;
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = yL_goal;
		goal->as<ob::SO2StateSpace::StateType>(2)->value = theta_goal;
		 
		// Create a problem instance
		auto pdef(std::make_shared<ob::ProblemDefinition>(si));
	 
		// Cet the start and goal states
		pdef->setStartAndGoalStates(start, goal, 0.1);
	 
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
	 
		// Attempt to solve within 5.0 * n sec (can be changed)
	
		
		solved = planner->ob::Planner::solve(5.0 * n);
		std::cout << "\n\nOutcome: " << solved << "\n\n"<< std::endl;
		
		if (solved == ob::PlannerStatus::EXACT_SOLUTION)
		{
			ob::PathPtr path = pdef->getSolutionPath();
			/*
			path->print(std::cout);
			auto numC = path->as<oc::PathControl>()->getControlCount();
			for (int i = 0; i < numC; i++)
			{
				cspace->as<oc::ControlSpace>()->printControl(path->as<oc::PathControl>()->getControl(i), std::cout);
			}			
			*/
			std::cout << "\n\nSolution found in " << n << "iteration(s)\n\n"<< std::endl;
			
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
    while(solved != ob::PlannerStatus::EXACT_SOLUTION || n < MAX_ITERATION);     
}




int main()
{
    plan();
    // print_vec(ctrl_list);
    return 0;
}
