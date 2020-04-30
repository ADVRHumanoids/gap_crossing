#include <vector>
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

#define PI 3.14159265
#define L 1

//Movement lenghts
#define DELTA_ROT PI/6
#define GAP_LENGTH L/10
#define SHORT_STEP 0.25
#define LONG_STEP 0.5
#define LATERAL_STEP 0.1
#define FEET_DIST_TOL 0.001
#define SHORT_STEP_DIST sqrt(pow(L, 2)+pow(SHORT_STEP, 2))
#define LONG_STEP_DIST sqrt(pow(L, 2)+pow(LONG_STEP, 2))
#define MAX_ITER 10

//Rotations
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


namespace ob = ompl::base;
namespace oc = ompl::control;


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

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    // Cast the start state to the desired type (consistent to the state space defined in plan())
    auto R2_R2_SO2state = start->as<ob::CompoundStateSpace::StateType>();
    
    // Extract x,y position of the right (R) and left foot(L) and orientation of the body
    auto posR = R2_R2_SO2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    auto posL = R2_R2_SO2state->as<ob::RealVectorStateSpace::StateType>(1)->values;
    auto theta = R2_R2_SO2state->as<ob::SO2StateSpace::StateType>(2)->value;
    
    double feet_dist = sqrt(pow((posR[0]-posL[0]),2) + pow((posR[1]-posL[1]),2));
    
    
    // Cast the control to the desired type (consistent to the control space defined in plan())
    auto ctrl = control->as<oc::DiscreteControlSpace::ControlType>()->value;

	auto r_result = result->as<ob::CompoundStateSpace::StateType>();
    
    double xR, yR, xL, yL;
    
    switch(ctrl)
    {
		case(CW_AROUND_R):
			if(feet_dist<L+FEET_DIST_TOL)
			{
				theta = mod2PI(theta + DELTA_ROT);
				xR = posR[0];
				yR = posR[1];
				xL = posR[0] + feet_dist * cos(theta); 
				yL = posR[1] + feet_dist * sin(theta);
			} 
			break;
			
		case(CCW_AROUND_R):
			if(feet_dist<L+FEET_DIST_TOL)
			{
				theta = mod2PI(theta - DELTA_ROT);
				xR = posR[0];
				yR = posR[1];
				xL = posR[0] + feet_dist * cos(theta); 
				yL = posR[1] + feet_dist * sin(theta);
			}
			break;
		
		case(CW_AROUND_L):
			if(feet_dist<L+FEET_DIST_TOL)
			{
				theta = mod2PI(theta + DELTA_ROT);
				xR = posL[0] - feet_dist * cos(theta); 
				yR = posL[1] - feet_dist * sin(theta);
				xL = posL[0];
				yL = posL[1];
			}
			break;
		
		case(CCW_AROUND_L):
			if(feet_dist<L+FEET_DIST_TOL)
			{
				theta = mod2PI(theta - DELTA_ROT);
				xR = posL[0] - feet_dist * cos(theta); 
				yR = posL[1] - feet_dist * sin(theta);
				xL = posL[0];
				yL = posL[1];
			}	
			break;
		
		case(SH_FW_R):
			xR = posR[0] + SHORT_STEP * sin(theta);
			yR = posR[1] - SHORT_STEP * cos(theta);
			xL = posL[0];
			yL = posL[1];
			break;
		
		case(SH_BW_R):
			xR = posR[0] - SHORT_STEP * sin(theta);
			yR = posR[1] + SHORT_STEP * cos(theta);
			xL = posL[0];
			yL = posL[1];
			break;
		
		case(SH_FW_L):
			xR = posR[0];
			yR = posR[1];
			xL = posL[0] + SHORT_STEP * sin(theta);
			yL = posL[1] - SHORT_STEP * cos(theta);
			break;
		
		case(SH_BW_L):
			xR = posR[0];
			yR = posR[1];
			xL = posL[0] - SHORT_STEP * sin(theta);
			yL = posL[1] + SHORT_STEP * cos(theta);
			break;
		
		case(LO_FW_R):
			xR = posR[0] + LONG_STEP * sin(theta);
			yR = posR[1] - LONG_STEP * cos(theta);
			xL = posL[0];
			yL = posL[1];
			break;
		
		case(LO_FW_L):
			xR = posR[0];
			yR = posR[1];
			xL = posL[0] + LONG_STEP * sin(theta);
			yL = posL[1] - LONG_STEP * cos(theta);
			break;
		
		case(R_LAT_R):
			if(feet_dist < SHORT_STEP_DIST + FEET_DIST_TOL)
			{
				xR = posR[0] - LATERAL_STEP * cos(theta);
				yR = posR[1] - LATERAL_STEP * sin(theta);
				xL = posL[0];
				yL = posL[1];
			}
			break;
		
		case(L_LAT_R):
			xR = posR[0] + LATERAL_STEP * cos(theta);
			yR = posR[1] + LATERAL_STEP * sin(theta);
			xL = posL[0];
			yL = posL[1];
			break;
		
		case(R_LAT_L):
			xR = posR[0];
			yR = posR[1];
			xL = posL[0] - LATERAL_STEP * cos(theta);
			yL = posL[1] - LATERAL_STEP * sin(theta);
			break;
		
		case(L_LAT_L):
			if(feet_dist < SHORT_STEP_DIST + FEET_DIST_TOL)
			{
				xR = posR[0];
				yR = posR[1];
				xL = posL[0] + LATERAL_STEP * cos(theta);
				yL = posL[1] + LATERAL_STEP * sin(theta);
			}
			break;
	}
	r_result->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = xR;
	r_result->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = yR;
	r_result->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = xL;
	r_result->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = yL;			
	r_result->as<ob::SO2StateSpace::StateType>(2)->value = theta;
				
}

// State validity function: returns true if the state sampled satisfies the bounds and if it is not inside a gap)
bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{        
    auto R2_R2_SO2state = state->as<ob::CompoundStateSpace::StateType>();
	
	auto posR = R2_R2_SO2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    auto posL = R2_R2_SO2state->as<ob::RealVectorStateSpace::StateType>(1)->values;
    auto theta = R2_R2_SO2state->as<ob::SO2StateSpace::StateType>(2)->value;
    
    double feet_dist = sqrt(pow((posR[0]-posL[0]),2) + pow((posR[1]-posL[1]),2));
    
    bool check_distance_min = feet_dist > L - FEET_DIST_TOL;
    bool check_distance_max = feet_dist < LONG_STEP_DIST + FEET_DIST_TOL; 
    
    bool check_gapR = posR[0] < -GAP_LENGTH/2 || posR[0] > GAP_LENGTH/2;
    bool check_gapL = posL[0] < -GAP_LENGTH/2 || posL[0] > GAP_LENGTH/2;
    
    bool check_obstacleR = posR[0] > 1.5 || posR[0] < 1 || posR[1] > 0.5 || posR[1] < -0.5; 
    bool check_obstacleL = posR[0] > 1.5 || posR[0] < 1 || posR[1] > 0.5 || posR[1] < -0.5; 

    return si->satisfiesBounds(state) && check_distance_min && check_distance_max && check_gapR && check_gapL && check_obstacleR && check_obstacleL;
}

void plan()
{
	int solved = -1;
	int n_iter = 0;
	do
	{
		n_iter++;
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
		
		// In the propagation of the tree, a new node is found applying exactly one primitive
		si->setMinMaxControlDuration(1, 1);

		// Set the state validity checker function
		si->setStateValidityChecker(
			[&si](const ob::State *state) { return isStateValid(si.get(), state); });
	 
		// Set the propagator
		si->setStatePropagator(propagate);

		// Create a start state
		ob::ScopedState<ob::CompoundStateSpace> start(space);
		
		double start_rx = -1;
		double start_ry = -0.5;
		double start_lx = -1;
		double start_ly = 0.5;
		double start_theta = atan2(start_ly - start_ry, start_lx - start_rx);
		
		start->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = start_rx; 		// xR_0
		start->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = start_ry;	 	// yR_0
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = start_lx; 		// xL_0 
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = start_ly;	 	// yL_0
		start->as<ob::SO2StateSpace::StateType>(2)->value = start_theta;
		

		// Create a goal state
		ob::ScopedState<ob::CompoundStateSpace> goal(space);
		
		double goal_rx = 2;
		double goal_ry = 0;
		double goal_lx = 3;
		double goal_ly = 0;
		double goal_theta = atan2(goal_ly - goal_ry, goal_lx - goal_rx);
		
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = goal_rx;
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = goal_ry;
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = goal_lx;  
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = goal_ly;
		goal->as<ob::SO2StateSpace::StateType>(2)->value = goal_theta;
		 
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
	 
		// Attempt to solve within 10 sec
		ob::PlannerStatus solved = planner->ob::Planner::solve(30.0);
		if(solved == ob::PlannerStatus::EXACT_SOLUTION)
		{
			ob::PathPtr path = pdef->getSolutionPath();
			path->print(std::cout); 
			auto numC = path->as<oc::PathControl>()->getControlCount();
			for(int i=0; i<numC; i++)
			{
				cspace->as<oc::ControlSpace>()->printControl(path->as<oc::PathControl>()->getControl(i),std::cout);
			}
			std::cout << "Found solution after " << n_iter << " iteration(s)." << std::endl;
			
			// Geerate files for plots
			std::ofstream log_file;
			log_file.open ("log.txt");
			path->print(log_file);
			log_file.close();
			std::ofstream path_file;
			path_file.open ("path.txt");
			// path->as<ompl::geometric::PathGeometric>()->printAsMatrix(path_file);
			path->as<ompl::control::PathControl>()->printAsMatrix(path_file);
			path_file.close();
			
			break;
		}
	    std::cout << "\n\n\nITERATION " << n_iter << "\nOutcome: " << solved << "\n\n" << std::endl;
	    std::cout << "-------------------------------------------------------------------------------------------------------\n" << std::endl;
	    if (n_iter == MAX_ITER)
	    {
			std::cout << "\n\n\n\n\n\nNO SOLUTION FOUND AFTER " << n_iter << " ITERATIONS. TRY GIVING MORE TIME." << std::endl;
			break;
		} 
	} while(solved != ob::PlannerStatus::EXACT_SOLUTION);	
}

int main()
{
    plan();
    return 0;
}
