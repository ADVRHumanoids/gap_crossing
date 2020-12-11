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
#define TIME		  300.0	

#define PI 		3.14159265
#define L_MIN 	0.75		// robot lenght
#define L_MAX 	1.25 		// robot lenght
#define W 		0.5 	    // robot width

//Movement lenghts
#define DELTA_ROT 		PI/36
#define DELTA_S			0.05
#define TOL 			0.001

//Rotations
#define CW  			0 //cloak-wise rotation of DELTA_ROT
#define CCW 			1 //countercloak-wise rotation of DELTA_ROT
//Rolls
#define FW_4 			2 //forward roll of the four wheels
#define BW_4 			3 //backward roll of the four wheels
#define FW_2B 			4 //forward roll of the two back wheels
#define BW_2B 			5 //backward roll of the two back wheels
#define FW_2F			6 //forward roll of the two front wheels
#define BW_2F 			7 //backward roll of the two front wheels



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
    
    // Extract x,y position of the back right (B) and front right (F) foot and orientation of the body
    auto posBr = R2_R2_SO2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    double xBr = posBr[0], yBr = posBr[1];
    auto posFr = R2_R2_SO2state->as<ob::RealVectorStateSpace::StateType>(1)->values;
    double xFr = posFr[0], yFr = posFr[1];
    auto phi = R2_R2_SO2state->as<ob::SO2StateSpace::StateType>(2)->value;
    
    const double posBl[2] = {posBr[0] - W * sin(phi), posBr[1] + W * cos(phi)};
    const double xBl = posBl[0], yBl = posBl[1];
    const double posFl[2] = {posFr[0] - W * sin(phi), posFr[1] + W * cos(phi)};
    const double xFl = posFl[0], yFl = posFl[1];
    
    auto xcenter = (xBr+xBl+xFr+xFl)/4, ycenter = (yBr+yBl+yFr+yFl)/4;
    
    // Cast the control to the desired type (consistent to the control space defined in plan())
    auto ctrl = control->as<oc::DiscreteControlSpace::ControlType>()->value;

	// Cast the result state to the desired type (consistent to the state space defined in plan())
    auto r_result = result->as<ob::CompoundStateSpace::StateType>();
  
	// Pick a control and propagate the state (x and y will be only propagated)
	double xBr_new, yBr_new, xFr_new, yFr_new, phi_new;
	double thetaB, rhoB, thetaF, rhoF;
    switch (ctrl)
    {
		case(CW):
			thetaB = atan2(yBr - ycenter, xBr - xcenter);
			rhoB = sqrt(pow((xBr - xcenter), 2) + pow((yBr - ycenter), 2));
			xBr_new = xcenter + rhoB*cos(thetaB - DELTA_ROT);
			yBr_new = ycenter + rhoB*sin(thetaB - DELTA_ROT);
			
			thetaF = atan2(yFr - ycenter, xFr - xcenter);
			rhoF = sqrt(pow((xFr - xcenter), 2) + pow((yFr - ycenter), 2));
			xFr_new = xcenter + rhoF*cos(thetaF - DELTA_ROT);
			yFr_new = ycenter + rhoF*sin(thetaF - DELTA_ROT);
			
			phi_new = atan2(yFr_new-yBr_new, xFr_new-xBr_new);
			break;
		case(CCW):
			thetaB = atan2(yBr - ycenter, xBr - xcenter);
			rhoB = sqrt(pow((xBr - xcenter), 2) + pow((yBr - ycenter), 2));
			xBr_new = xcenter + rhoB*cos(thetaB + DELTA_ROT);
			yBr_new = ycenter + rhoB*sin(thetaB + DELTA_ROT);
			
			thetaF = atan2(yFr - ycenter, xFr - xcenter);
			rhoF = sqrt(pow((xFr - xcenter), 2) + pow((yFr - ycenter), 2));
			xFr_new = xcenter + rhoF*cos(thetaF + DELTA_ROT);
			yFr_new = ycenter + rhoF*sin(thetaF + DELTA_ROT);
			
			phi_new = atan2(yFr_new-yBr_new, xFr_new-xBr_new);
			break;
		case(FW_4):
			phi_new = phi;
			xBr_new = xBr + DELTA_S * cos(phi_new);
			yBr_new = yBr + DELTA_S * sin(phi_new);
			xFr_new = xFr + DELTA_S * cos(phi_new);
			yFr_new = yFr + DELTA_S * sin(phi_new);
			break;
		case(BW_4):
			phi_new = phi;
			xBr_new = xBr - DELTA_S * cos(phi_new);
			yBr_new = yBr - DELTA_S * sin(phi_new);
			xFr_new = xFr - DELTA_S * cos(phi_new);
			yFr_new = yFr - DELTA_S * sin(phi_new);
			break;	
			
		case(FW_2B):
			phi_new = phi;
			xBr_new = xBr + DELTA_S * cos(phi_new);
			yBr_new = yBr + DELTA_S * sin(phi_new);
			xFr_new = xFr;
			yFr_new = yFr;
			break;
		
		case(BW_2B):
			phi_new = phi;
			xBr_new = xBr - DELTA_S * cos(phi_new);
			yBr_new = yBr - DELTA_S * sin(phi_new);
			xFr_new = xFr;
			yFr_new = yFr;
			break;
			
		case(FW_2F):
			phi_new = phi;
			xBr_new = xBr;
			yBr_new = yBr;
			xFr_new = xFr + DELTA_S * cos(phi_new);
			yFr_new = yFr + DELTA_S * sin(phi_new);
			break;
			
		case(BW_2F):
			phi_new = phi;
			xBr_new = xBr;
			yBr_new = yBr;
			xFr_new = xFr - DELTA_S * cos(phi_new);
			yFr_new = yFr - DELTA_S * sin(phi_new);
			break;
			
	}
	
	r_result->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = xBr_new;
	r_result->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = yBr_new;
	r_result->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = xFr_new;
	r_result->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = yFr_new;
	r_result->as<ob::SO2StateSpace::StateType>(2)->value = phi_new;
}





// State validity function: returns true if the state sampled satisfies the bounds and if it is not inside a gap)
bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{   
	// Cast the start state to the desired type (consistent to the state space defined in plan())
    auto R2_R2_SO2state = state->as<ob::CompoundStateSpace::StateType>();
    
    // Extract x,y position of the right (R) and left foot(L) and orientation of the body
    auto posBr = R2_R2_SO2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    auto posFr = R2_R2_SO2state->as<ob::RealVectorStateSpace::StateType>(1)->values;
    auto phi = R2_R2_SO2state->as<ob::SO2StateSpace::StateType>(2)->value;
    
    double length = sqrt(pow(posBr[0]-posFr[0],2) + pow(posBr[1]-posFr[1],2));
    
    const double posBl[2] = {posBr[0] - W * sin(phi), posBr[1] - W * cos(phi)};
    const double posFl[2] = {posFr[0] - W * sin(phi), posFr[1] - W * cos(phi)};
	
	bool check_bounds_r = si->satisfiesBounds(state);
	
	bool check_bounds_Bl = posBl[0] < 20 && posBl[0] > -20 && posBl[1] < 20 && posBl[1] > -20;
	bool check_bounds_Fl = posFl[0] < 20 && posFl[0] > -20 && posFl[1] < 20 && posFl[1] > -20;
	bool check_bounds_l = check_bounds_Bl && check_bounds_Fl;
	
	bool check_length = length > L_MIN-TOL && length < L_MAX + TOL;
	
	double gap_min_x = 1.95, gap_max_x = 3.05, gap_min_y = -1.05, gap_max_y = 1.05;
	
	bool check_gap_Br = posBr[1] < gap_min_y || posBr[1] > gap_max_y || posBr[0] < gap_min_x || posBr[0] > gap_max_x;
	bool check_gap_Fr = posFr[1] < gap_min_y || posFr[1] > gap_max_y || posFr[0] < gap_min_x || posFr[0] > gap_max_x;
	bool check_gap_Bl = posBl[1] < gap_min_y || posBl[1] > gap_max_y || posBl[0] < gap_min_x || posBl[0] > gap_max_x;
	bool check_gap_Fl = posFl[1] < gap_min_y || posFl[1] > gap_max_y || posFl[0] < gap_min_x || posFl[0] > gap_max_x;
	
	bool check_gap = check_gap_Br && check_gap_Fr && check_gap_Bl && check_gap_Fl;
	
	
	//bool check_obsR = posR[0] < 1 || posR[0] > 1.5 || posR[1] < -0.5 || posR[1] > 0.5;
	//bool check_obsL = posL[0] < 1 || posL[0] > 1.5 || posL[1] < -0.5 || posL[1] > 0.5;
	
	//std::cout << "----" << std::endl;
	//std::cout << "check_bounds: " << check_bounds << std::endl;
	//std::cout << "check_dist_min: " << check_dist_min << std::endl;
	//std::cout << "check_dist_max: " << check_dist_max << std::endl;
	
    return check_bounds_r && check_bounds_l && check_length && check_gap;
}





void plan()
{
	ob::PlannerStatus solved = ob::PlannerStatus::UNKNOWN;
	int n = 0;
	do {
		// Create the state space
		auto R2B = std::make_shared<ob::RealVectorStateSpace>(2);
		auto R2F = std::make_shared<ob::RealVectorStateSpace>(2);
		
		ob::RealVectorBounds bounds(2);
		bounds.setLow(0, -20);
		bounds.setLow(1, -20);
		bounds.setHigh(0, 20);
		bounds.setHigh(1, 20);
		R2B->setBounds(bounds);
		R2F->setBounds(bounds);
			
		auto SO2 = std::make_shared<ob::SO2StateSpace>();
		
		std::vector<ob::StateSpacePtr> components {R2B, R2F, SO2};
		std::vector<double> weights {1.0, 1.0, 1.0};
		auto space = std::make_shared<ob::CompoundStateSpace>(components, weights);
		
		// Create the control space
		auto cspace = std::make_shared<ompl::control::DiscreteControlSpace>(space, 0, 7);
		
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
		
		double xB_start = 0.0, yB_start = 0.0, xF_start = 1.0, yF_start = 0.0;
		double phi_start = atan2(yF_start-yB_start, xF_start-xB_start);
		
		start->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = xB_start;
		start->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = yB_start;
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = xF_start;
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = yF_start;
		start->as<ob::SO2StateSpace::StateType>(2)->value = phi_start;
		

		// Create a goal state
		ob::ScopedState<ob::CompoundStateSpace> goal(space);
		
		double xB_goal = 4.0, yB_goal = 0.0, xF_goal = 5.0, yF_goal = 0.0;
		double phi_goal = atan2(yF_goal-yB_goal, xF_goal-xB_goal);
		
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = xB_goal;
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = yB_goal;
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = xF_goal;
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = yF_goal;
		goal->as<ob::SO2StateSpace::StateType>(2)->value = phi_goal;
		 
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
    // print_vec(ctrl_list);
    return 0;
}
