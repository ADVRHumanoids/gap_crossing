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

#define PI 			3.14159265

// PROBLEM DATA
#define MAX_ITER 	1		// number of iteration completed if EXACT_SOLUTION is not found before
#define TIME		20.0	// time given for each iteration
#define TOLERANCE 	0.20	// tolerance for considering a solution EXACT or not

// ENVIRONMENT
#define OBS_LENGTH	0.4
#define OBS_WIDTH	0.2

// MOVEMENT LENGTHS
#define ROLL	 		0.05
#define STEP 			0.30
#define DELTA_ROT 		PI/36
#define TOL			 	0.001

// ROBOT STRUCTURE
#define L 	 1
#define Lmin L - STEP								// minimum distance between front and back
#define Lmax L + STEP								// maximum distance between front and back
#define Wmin 0.5									// nominal distance between right and left
#define Wmax sqrt(pow(Wmin, 2) + pow(STEP, 2))		// maximum distance between right and left (due to step)

// PRIMITIVES
// Rotations
#define CW  	0 	//clockwise rotation of DELTA_ROT
#define CCW 	1 	//counter-clockwise rotation of DELTA_ROT
// Rolling
#define FW_4 	2 	//forward rolling of DELTA_S with four wheels 
#define BW_4 	3 	//backward rolling of DELTA_S with four wheels
#define FW_2B 	4 	//forward rolling of DELTA_S with the two front wheels
#define BW_2B 	5 	//backward rolling of DELTA_S with the two front wheels
#define FW_2F 	6 	//forward rolling of DELTA_S with the two back wheels
#define BW_2F 	7 	//backward rolling of DELTA_S with the two back wheels
#define R_4		8	//right rolling of DELTA_S with four wheels
#define L_4		9	//left rolling of DELTA_S with four wheels
// Steps
#define BR_FWS	10	//forward step with BR foot
#define BL_FWS	11	//forward step with BL foot
#define FR_FWS	12  //forward step with FR foot
#define FL_FWS	13	//forward step with FL foot
#define BR_BWS	14	//backward step with BR foot
#define BL_BWS	15	//backward step with BL foot
#define FR_BWS	16  //backward step with FR foot
#define FL_BWS	17	//backward step with FL foot



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
    auto R2_4_state = start->as<ob::CompoundStateSpace::StateType>();
    
    // Extract x,y position of the back right (B) and front right foot(F) and orientation of the body
    auto posBR = R2_4_state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    auto posBL = R2_4_state->as<ob::RealVectorStateSpace::StateType>(1)->values;
    auto posFR = R2_4_state->as<ob::RealVectorStateSpace::StateType>(2)->values;
    auto posFL = R2_4_state->as<ob::RealVectorStateSpace::StateType>(3)->values;
   
    double xBR = posBR[0];
    double yBR = posBR[1];
    double xBL = posBL[0];
    double yBL = posBL[1];
    double xFR = posFR[0];
    double yFR = posFR[1];
    double xFL = posFL[0];
    double yFL = posFL[1];
    
    double phi = atan2(yFR - yBR, xFR - xBR);
    double alphaB = atan2(yBL - yBR, xBL - xBR);
    double alphaF = atan2(yFL - yFR, xFL - xFR);
    bool isRectangle = (fabs(alphaB - phi) > PI/2 - 0.01) && (fabs(alphaB - phi) < PI/2 + 0.01) && (fabs(alphaF - phi) > PI/2 - 0.01) && (fabs(alphaF - phi) < PI/2 + 0.01);
    double xCenter = (xBR + xBL + xFR + xFL)/4;
    double yCenter = (yBR + yBL + yFR + yFL)/4;
        
    double thetaBR, rhoBR, thetaBL, rhoBL, thetaFR, rhoFR, thetaFL, rhoFL;
   
    // Cast the control to the desired type (consistent to the control space defined in plan())
    auto ctrl = control->as<oc::DiscreteControlSpace::ControlType>()->value;

	auto r_result = result->as<ob::CompoundStateSpace::StateType>();
    
    switch(ctrl)
    {
		case(CW):
			if(isRectangle)
			{
				thetaBR = atan2(yBR - yCenter, xBR - xCenter);
				rhoBR = sqrt(pow(xBR - xCenter, 2) + pow(yBR - yCenter, 2));
				xBR = xCenter + rhoBR * cos(thetaBR - DELTA_ROT);
				yBR = yCenter + rhoBR * sin(thetaBR - DELTA_ROT);
				
				thetaBL = atan2(yBL - yCenter, xBL - xCenter);
				rhoBL = sqrt(pow(xBL - xCenter, 2) + pow(yBL - yCenter, 2));
				xBL = xCenter + rhoBL * cos(thetaBL - DELTA_ROT);
				yBL = yCenter + rhoBL * sin(thetaBL - DELTA_ROT);
				
				thetaFR = atan2(yFR - yCenter, xFR - xCenter);
				rhoFR = sqrt(pow(xFR - xCenter, 2) + pow(yFR - yCenter, 2));
				xFR = xCenter + rhoFR * cos(thetaFR - DELTA_ROT);
				yFR = yCenter + rhoFR * sin(thetaFR - DELTA_ROT);
										
				thetaFL = atan2(yFL - yCenter, xFL - xCenter);
				rhoFL = sqrt(pow(xFL - xCenter, 2) + pow(yFL - yCenter, 2));
				xFL = xCenter + rhoFL * cos(thetaFL - DELTA_ROT);
				yFL = yCenter + rhoFL * sin(thetaFL - DELTA_ROT);
			}
			break;
			
		case(CCW):
			if(isRectangle)
			{
				thetaBR = atan2(yBR - yCenter, xBR - xCenter);
				rhoBR = sqrt(pow(xBR - xCenter, 2) + pow(yBR - yCenter, 2));
				xBR = xCenter + rhoBR * cos(thetaBR + DELTA_ROT);
				yBR = yCenter + rhoBR * sin(thetaBR + DELTA_ROT);
				
				thetaBL = atan2(yBL - yCenter, xBL - xCenter);
				rhoBL = sqrt(pow(xBL - xCenter, 2) + pow(yBL - yCenter, 2));
				xBL = xCenter + rhoBL * cos(thetaBL + DELTA_ROT);
				yBL = yCenter + rhoBL * sin(thetaBL + DELTA_ROT);
				
				thetaFR = atan2(yFR - yCenter, xFR - xCenter);
				rhoFR = sqrt(pow(xFR - xCenter, 2) + pow(yFR - yCenter, 2));
				xFR = xCenter + rhoFR * cos(thetaFR + DELTA_ROT);
				yFR = yCenter + rhoFR * sin(thetaFR + DELTA_ROT);
				
				thetaFL = atan2(yFL - yCenter, xFL - xCenter);
				rhoFL = sqrt(pow(xFL - xCenter, 2) + pow(yFL - yCenter, 2));
				xFL = xCenter + rhoFL * cos(thetaFL + DELTA_ROT);
				yFL = yCenter + rhoFL * sin(thetaFL + DELTA_ROT);
			}
			break;
		
		case(FW_4):
			if(isRectangle)
			{
				xBR = xBR + ROLL*cos(phi);
				yBR = yBR + ROLL*sin(phi);
				xBL = xBL + ROLL*cos(phi);
				yBL = yBL + ROLL*sin(phi);
				xFR = xFR + ROLL*cos(phi);
				yFR = yFR + ROLL*sin(phi);
				xFL = xFL + ROLL*cos(phi);
				yFL = yFL + ROLL*sin(phi);
			}
			break;
		
		case(BW_4):
			if(isRectangle)
			{
				xBR = xBR - ROLL*cos(phi);
				yBR = yBR - ROLL*sin(phi);
				xBL = xBL - ROLL*cos(phi);
				yBL = yBL - ROLL*sin(phi);
				xFR = xFR - ROLL*cos(phi);
				yFR = yFR - ROLL*sin(phi);
				xFL = xFL - ROLL*cos(phi);
				yFL = yFL - ROLL*sin(phi);
			}
			break;
			
		case(FW_2B):
			if(isRectangle)
			{
				xBR = xBR + ROLL*cos(phi);
				yBR = yBR + ROLL*sin(phi);
				xBL = xBL + ROLL*cos(phi);
				yBL = yBL + ROLL*sin(phi);
			}
			break;
		
		case(BW_2B):
			if(isRectangle)
			{
				xBR = xBR - ROLL*cos(phi);
				yBR = yBR - ROLL*sin(phi);
				xBL = xBL - ROLL*cos(phi);
				yBL = yBL - ROLL*sin(phi);
			}
			break;	
		
		case(FW_2F):
			if(isRectangle)
			{
				xFR = xFR + ROLL*cos(phi);
				yFR = yFR + ROLL*sin(phi);
				xFL = xFL + ROLL*cos(phi);
				yFL = yFL + ROLL*sin(phi);
			}
			break;
		
		case(BW_2F):
			if(isRectangle)
			{
				xFR = xFR - ROLL*cos(phi);
				yFR = yFR - ROLL*sin(phi);
				xFL = xFL - ROLL*cos(phi);
				yFL = yFL - ROLL*sin(phi);
			}
			break;	
			
		case(R_4):
			if(isRectangle)
			{
				xBR = xBR + ROLL*sin(phi);
				yBR = yBR - ROLL*cos(phi);
				xBL = xBL + ROLL*sin(phi);
				yBL = yBL - ROLL*cos(phi);
				xFR = xFR + ROLL*sin(phi);
				yFR = yFR - ROLL*cos(phi);
				xFL = xFL + ROLL*sin(phi);
				yFL = yFL - ROLL*cos(phi);
			}
		
		case(L_4):
			if(isRectangle)
			{
				xBR = xBR - ROLL*sin(phi);
				yBR = yBR + ROLL*cos(phi);
				xBL = xBL - ROLL*sin(phi);
				yBL = yBL + ROLL*cos(phi);
				xFR = xFR - ROLL*sin(phi);
				yFR = yFR + ROLL*cos(phi);
				xFL = xFL - ROLL*sin(phi);
				yFL = yFL + ROLL*cos(phi);
			}
		
		case(BR_FWS):
			xBR = xBR + STEP*cos(phi);
			yBR = yBR + STEP*sin(phi);
			break;
			
		case(BL_FWS):
			xBL = xBL + STEP*cos(phi);
			yBL = yBL + STEP*sin(phi);
			break;
		
		case(FR_FWS):
			xFR = xFR + STEP*cos(phi);
			yFR = yFR + STEP*sin(phi);
			break;
		
		case(FL_FWS):
			xFL = xFL + STEP*cos(phi);
			yFL = yFL + STEP*sin(phi);
			break;
			
		case(BR_BWS):
			xBR = xBR - STEP*cos(phi);
			yBR = yBR - STEP*sin(phi);
			break;
			
		case(BL_BWS):
			xBL = xBL - STEP*cos(phi);
			yBL = yBL - STEP*sin(phi);
			break;
		
		case(FR_BWS):
			xFR = xFR - STEP*cos(phi);
			yFR = yFR - STEP*sin(phi);
			break;
		
		case(FL_BWS):
			xFL = xFL - STEP*cos(phi);
			yFL = yFL - STEP*sin(phi);
			break;
	}
	r_result->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = xBR;
	r_result->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = yBR;
	r_result->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = xBL;
	r_result->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = yBL;			
	r_result->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = xFR;
	r_result->as<ob::RealVectorStateSpace::StateType>(2)->values[1] = yFR;
	r_result->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = xFL;
	r_result->as<ob::RealVectorStateSpace::StateType>(3)->values[1] = yFL;					
}

// State validity function: returns true if the state sampled satisfies the bounds and if it is not inside a gap)
bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{      
    // Cast the start state to the desired type (consistent to the state space defined in plan())
    auto R2_4_state = state->as<ob::CompoundStateSpace::StateType>();
    
    // Extract x,y position of the back right (B) and front right foot(F) and orientation of the body
    auto posBR = R2_4_state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    auto posBL = R2_4_state->as<ob::RealVectorStateSpace::StateType>(1)->values;
    auto posFR = R2_4_state->as<ob::RealVectorStateSpace::StateType>(2)->values;
    auto posFL = R2_4_state->as<ob::RealVectorStateSpace::StateType>(3)->values;
   
    double xBR = posBR[0];
    double yBR = posBR[1];
    double xBL = posBL[0];
    double yBL = posBL[1];
    double xFR = posFR[0];
    double yFR = posFR[1];
    double xFL = posFL[0];
    double yFL = posFL[1];
    
    double lengthR = sqrt(pow((posBR[0]-posFR[0]),2) + pow((posBR[1]-posFR[1]),2));
    double lengthL = sqrt(pow((posBL[0]-posFL[0]),2) + pow((posBL[1]-posFL[1]),2));
    double widthB = sqrt(pow((posBR[0]-posBL[0]),2) + pow((posBR[1]-posBL[1]),2));
    double widthF = sqrt(pow((posFR[0]-posFL[0]),2) + pow((posFR[1]-posFL[1]),2));

	bool check_lengthR_min = lengthR > Lmin - TOL;
    bool check_lengthR_max = lengthR < Lmax + TOL;
    bool check_lengthL_min = lengthL > Lmin - TOL;
    bool check_lengthL_max = lengthL < Lmax + TOL; 
    bool check_length = check_lengthR_min && check_lengthR_max && check_lengthL_min && check_lengthL_max;
    
    bool check_widthB_min = widthB > Wmin - TOL;
    bool check_widthB_max = widthB < Wmax + TOL;
    bool check_widthF_min = widthF > Wmin - TOL;
    bool check_widthF_max = widthF < Wmax + TOL; 
	bool check_width = check_widthB_min && check_widthB_max && check_widthF_min && check_widthF_max;
		
	bool check_geometry = check_length && check_width;
    bool check_bounds = si->satisfiesBounds(state);  
   
    
    double gap_min_x = 1.45;
    double gap_max_x = 1.55;
    
    bool BR_gap =  posBR[0] < gap_min_x || posBR[0] > gap_max_x;
    bool BL_gap = posBL[0] < gap_min_x || posBL[0] > gap_max_x;
    bool FR_gap = posFR[0] < gap_min_x || posFR[0] > gap_max_x;
    bool FL_gap = posFL[0] < gap_min_x || posFL[0] > gap_max_x;
    bool check_gap = BR_gap && BL_gap && FR_gap && FL_gap;

    return check_bounds && check_geometry && check_gap;
}

void plan()
{
	int solved = -1;
	int n_iter = 0;
	do
	{
		n_iter++;
		// Create the state space
		auto R2BR = std::make_shared<ob::RealVectorStateSpace>(2);
		auto R2BL = std::make_shared<ob::RealVectorStateSpace>(2);
		auto R2FR = std::make_shared<ob::RealVectorStateSpace>(2);
		auto R2FL = std::make_shared<ob::RealVectorStateSpace>(2);
		
		ob::RealVectorBounds bounds(2);
		bounds.setLow(0, -10);
		bounds.setLow(1, -10);
		bounds.setHigh(0, 10);
		bounds.setHigh(1, 10);
		R2BR->setBounds(bounds);
		R2BL->setBounds(bounds);
		R2FR->setBounds(bounds);
		R2FL->setBounds(bounds);
			
		std::vector<ob::StateSpacePtr> components {R2BR, R2BL, R2FR, R2FL};
		std::vector<double> weights {1.0, 1.0, 1.0, 1.0};
		auto space = std::make_shared<ob::CompoundStateSpace>(components, weights);
		
		// Create the control space
		auto cspace = std::make_shared<ompl::control::DiscreteControlSpace>(space, 0, 17);
	
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
		
		double start_brx = 0;
		double start_bry = 0;
		double start_blx = 0;
		double start_bly = 0.5;
		double start_frx = 1;
		double start_fry = 0;
		double start_flx = 1;
		double start_fly = 0.5;
		
		start->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = start_brx; 		// xBR_0
		start->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = start_bry;	 	// yBR_0
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = start_blx; 		// xBL_0 
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = start_bly;	 	// yBL_0
		start->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = start_frx; 		// xFR_0
		start->as<ob::RealVectorStateSpace::StateType>(2)->values[1] = start_fry;	 	// yFR_0
		start->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = start_flx; 		// xFL_0 
		start->as<ob::RealVectorStateSpace::StateType>(3)->values[1] = start_fly;	 	// yFL_0
		
		// Create a goal state
		ob::ScopedState<ob::CompoundStateSpace> goal(space);
		
		double goal_brx = 3;
		double goal_bry = 0.5;
		double goal_blx = 3;
		double goal_bly = 1;
		double goal_frx = 4;
		double goal_fry = 0.5;
		double goal_flx = 4;
		double goal_fly = 1;
		
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = goal_brx;
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = goal_bry;
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = goal_blx;  
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = goal_bly;
		goal->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = goal_frx;
		goal->as<ob::RealVectorStateSpace::StateType>(2)->values[1] = goal_fry;
		goal->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = goal_flx;  
		goal->as<ob::RealVectorStateSpace::StateType>(3)->values[1] = goal_fly;
		 
		// Create a problem instance
		auto pdef(std::make_shared<ob::ProblemDefinition>(si));
	 
		// Cet the start and goal states
		pdef->setStartAndGoalStates(start, goal, TOLERANCE);
	 
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
	 
		// Attempt to solve within TIME sec
		ob::PlannerStatus solved = planner->ob::Planner::solve(TIME);
		if(solved == ob::PlannerStatus::EXACT_SOLUTION || n_iter == MAX_ITER)
		{
			ob::PathPtr path = pdef->getSolutionPath();
			path->print(std::cout); 
			
			std::cout << "\n\n\nITERATION " << n_iter << "\nOutcome: " << solved << "\n\n" << std::endl;
			std::cout << "-------------------------------------------------------------------------------------------------------" << std::endl;
			std::cout << "-------------------------------------------------------------------------------------------------------\n" << std::endl;
			
			auto numC = path->as<oc::PathControl>()->getControlCount();
			std::cout << "\nCONTROL LIST: \n" << std::endl;
			for(int i=0; i<numC; i++)
			{
				cspace->as<oc::ControlSpace>()->printControl(path->as<oc::PathControl>()->getControl(i),std::cout);
			}		
			
			// Generate files for plots
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
		else
		{
			std::cout << "\n\n\nITERATION " << n_iter << "\nOutcome: " << solved << "\n\n" << std::endl;
			std::cout << "-------------------------------------------------------------------------------------------------------" << std::endl;
			std::cout << "-------------------------------------------------------------------------------------------------------\n" << std::endl;
		}
	} while(solved != ob::PlannerStatus::EXACT_SOLUTION && n_iter < MAX_ITER);	
}

int main()
{
    plan();
    return 0;
}
