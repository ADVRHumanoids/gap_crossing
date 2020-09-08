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
#include <cstdio>      
#include <iostream>
#include <fstream>

//vedi SST

#define MAX_ITERATION 	10
#define TIME		  	5.0	
#define THRESHOLD		0.1

#define PI 				3.14159265


//Movement lenghts
#define DELTA_ROT 		10*(PI/180)
#define ROL				0.05
#define STEP			0.20

#define TOL 			0.001
#define ACTIVATION_STEP_TH 0.05

//Environment
#define X_GAP_MIN 		2.215
#define X_GAP_MAX 		2.365
#define Y_GAP_MIN		-1.55
#define Y_GAP_MAX		0.35

#define X_OBS_MIN		0.65
#define X_OBS_MAX		1.65
#define Y_OBS_MIN		-0.35
#define Y_OBS_MAX		0.35

//Robot geometry
#define L				0.70
#define L_MIN 			L-STEP		
#define L_MAX 			L+STEP 		
#define W_MIN			0.70	    
#define W_MAX			sqrt(pow(W_MIN, 2)+ pow(STEP, 2))

//Rotations
#define CW  			0 //clockwise spin of DELTA_ROT
#define CCW 			1 //counter-clockwise spin of DELTA_ROT
//Rolls
#define FW_4 			2 //forward roll of the four wheels of ROL
#define BW_4 			3 //backward roll of the four wheels of ROL
#define R_4				4 //right roll of the four wheels of ROL
#define L_4				5 //left roll of the four wheels of ROL
//Steps
#define FW_S_BR			6 //forward step of back right foot
#define FW_S_BL			7 //forward step of back left left
#define FW_S_FR			8 //forward step of front right foot
#define FW_S_FL			9 //forward step of front left foot



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
void propagateWhileValid(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    // Cast the start state to the desired type (consistent to the state space defined in plan())
    auto R2_4state = start->as<ob::CompoundStateSpace::StateType>();
    
    // Extract x,y position of the back right (B) and front right (F) foot and orientation of the body
    auto posBR = R2_4state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    double xBR = posBR[0], yBR = posBR[1];
    auto posBL = R2_4state->as<ob::RealVectorStateSpace::StateType>(1)->values;
    double xBL = posBL[0], yBL = posBL[1];
    auto posFR = R2_4state->as<ob::RealVectorStateSpace::StateType>(2)->values;
    double xFR = posFR[0], yFR = posFR[1];
    auto posFL = R2_4state->as<ob::RealVectorStateSpace::StateType>(3)->values;
    double xFL = posFL[0], yFL = posFL[1];
    
    double phi = atan2(yFR - yBR, xFR - xBR);
    
    double l_B = sqrt(pow(xBR - xBL, 2) + pow(yBR - yBL, 2));
    double l_F = sqrt(pow(xFR - xFL, 2) + pow(yFR - yFL, 2));
    double l_R = sqrt(pow(xBR - xFR, 2) + pow(yBR - yFR, 2));
    double l_L = sqrt(pow(xBL - xFL, 2) + pow(yBL - yFL, 2));
    
    double alphaB = atan2(yBL - yBR, xBL - xBR);
    double alphaF = atan2(yFL - yFR, xFL - xFR);
    
    bool isRectangle = fabs(phi - alphaB) >= (PI/2 - TOL) && fabs(phi - alphaB) <= (PI/2 + TOL) && fabs(phi - alphaF) >= (PI/2 - TOL) && fabs(phi - alphaF) <= (PI/2 + TOL);
    //bool isSquare = fabs(l_B - l_F) <= TOL && fabs(l_R - l_L) <= TOL && fabs(l_R - l_B) <= TOL;
    
    bool canSpin_xmin = xBR < X_GAP_MIN && xBL < X_GAP_MIN && xFR < X_GAP_MIN && xFL < X_GAP_MIN;
    bool canSpin_xmax = xBR > X_GAP_MAX && xBL > X_GAP_MAX && xFR > X_GAP_MAX && xFL > X_GAP_MAX;
    bool canSpin_ymin = yBR < Y_GAP_MIN && yBL < Y_GAP_MIN && yFR < Y_GAP_MIN && yFL < Y_GAP_MIN;
    bool canSpin_ymax = yBR > Y_GAP_MAX && yBL > Y_GAP_MAX && yFR > Y_GAP_MAX && yFL > Y_GAP_MAX;
    
    bool canSpin = canSpin_xmin || canSpin_xmax || canSpin_ymin || canSpin_ymax;
    
    bool canStepFR = 0 < (X_GAP_MIN - xFR) && (X_GAP_MIN - xFR) < ACTIVATION_STEP_TH && Y_GAP_MIN < yFR && yFR < Y_GAP_MAX;
    bool canStepFL = 0 < (X_GAP_MIN - xFL) && (X_GAP_MIN - xFL) < ACTIVATION_STEP_TH && Y_GAP_MIN < yFL && yFL < Y_GAP_MAX;
    bool canStepBR = 0 < (X_GAP_MIN - xBR) && (X_GAP_MIN - xBR) < ACTIVATION_STEP_TH && Y_GAP_MIN < yBR && yBR < Y_GAP_MAX;
    bool canStepBL = 0 < (X_GAP_MIN - xBL) && (X_GAP_MIN - xBL) < ACTIVATION_STEP_TH && Y_GAP_MIN < yBL && yBL < Y_GAP_MAX;
    
    //bool canStepFR = true;
    //bool canStepFL = true;
    //bool canStepBR = true;
    //bool canStepBL = true;
    
    auto xcenter = (xBR+xBL+xFR+xFL)/4, ycenter = (yBR+yBL+yFR+yFL)/4;
    
    // Cast the control to the desired type (consistent to the control space defined in plan())
    auto ctrl = control->as<oc::DiscreteControlSpace::ControlType>()->value;

	// Cast the result state to the desired type (consistent to the state space defined in plan())
    auto r_result = result->as<ob::CompoundStateSpace::StateType>();
  
	// Pick a control and propagate the state (x and y will be only propagated)
	double xBR_new = xBR, yBR_new = yBR, xBL_new = xBL, yBL_new = yBL, xFR_new = xFR, yFR_new = yFR, xFL_new = xFL, yFL_new = yFL;
	double thetaBR, rhoBR, thetaBL, rhoBL, thetaFR, rhoFR, thetaFL, rhoFL;
	bool moved = false;
	
    switch (ctrl)
    {
		// clockwise rotation of DELTA_ROT
		case(CW):
			if (canSpin){
				moved = true;
				
				thetaBR = atan2(yBR - ycenter, xBR - xcenter);
				rhoBR = sqrt(pow((xBR - xcenter), 2) + pow((yBR - ycenter), 2));
				xBR_new = xcenter + rhoBR*cos(thetaBR - DELTA_ROT);
				yBR_new = ycenter + rhoBR*sin(thetaBR - DELTA_ROT);
				
				thetaBL = atan2(yBL - ycenter, xBL - xcenter);
				rhoBL = sqrt(pow((xBL - xcenter), 2) + pow((yBL - ycenter), 2));
				xBL_new = xcenter + rhoBL*cos(thetaBL - DELTA_ROT);
				yBL_new = ycenter + rhoBL*sin(thetaBL - DELTA_ROT);
				
				thetaFR = atan2(yFR - ycenter, xFR - xcenter);
				rhoFR = sqrt(pow((xFR - xcenter), 2) + pow((yFR - ycenter), 2));
				xFR_new = xcenter + rhoFR*cos(thetaFR - DELTA_ROT);
				yFR_new = ycenter + rhoFR*sin(thetaFR - DELTA_ROT);
				
				thetaFL = atan2(yFL - ycenter, xFL - xcenter);
				rhoFL = sqrt(pow((xFL - xcenter), 2) + pow((yFL - ycenter), 2));
				xFL_new = xcenter + rhoFL*cos(thetaFL - DELTA_ROT);
				yFL_new = ycenter + rhoFL*sin(thetaFL - DELTA_ROT);
			}
			break;
		
		//counter-clockwise rotation of DELTA_ROT
		case(CCW):
			if(canSpin)
			{
				moved = true;
				
				thetaBR = atan2(yBR - ycenter, xBR - xcenter);
				rhoBR = sqrt(pow((xBR - xcenter), 2) + pow((yBR - ycenter), 2));
				xBR_new = xcenter + rhoBR*cos(thetaBR + DELTA_ROT);
				yBR_new = ycenter + rhoBR*sin(thetaBR + DELTA_ROT);
				
				thetaBL = atan2(yBL - ycenter, xBL - xcenter);
				rhoBL = sqrt(pow((xBL - xcenter), 2) + pow((yBL - ycenter), 2));
				xBL_new = xcenter + rhoBL*cos(thetaBL + DELTA_ROT);
				yBL_new = ycenter + rhoBL*sin(thetaBL + DELTA_ROT);
				
				thetaFR = atan2(yFR - ycenter, xFR - xcenter);
				rhoFR = sqrt(pow((xFR - xcenter), 2) + pow((yFR - ycenter), 2));
				xFR_new = xcenter + rhoFR*cos(thetaFR + DELTA_ROT);
				yFR_new = ycenter + rhoFR*sin(thetaFR + DELTA_ROT);
				
				thetaFL = atan2(yFL - ycenter, xFL - xcenter);
				rhoFL = sqrt(pow((xFL - xcenter), 2) + pow((yFL - ycenter), 2));
				xFL_new = xcenter + rhoFL*cos(thetaFL + DELTA_ROT);
				yFL_new = ycenter + rhoFL*sin(thetaFL + DELTA_ROT);
			}
			break;
		//forward roll of the four wheels of ROL
		case(FW_4):
			if(isRectangle)
			{
				moved = true;
				
				xBR_new = xBR + ROL * cos(phi);
				yBR_new = yBR + ROL * sin(phi);
				
				xBL_new = xBL + ROL * cos(phi);
				yBL_new = yBL + ROL * sin(phi);
				
				xFR_new = xFR + ROL * cos(phi);
				yFR_new = yFR + ROL * sin(phi);
				
				xFL_new = xFL + ROL * cos(phi);
				yFL_new = yFL + ROL * sin(phi);
			}
			break;
		//backward roll of the four wheels of ROL
		case(BW_4):
			if(isRectangle)
			{
				moved = true;
				
				xBR_new = xBR - ROL * cos(phi);
				yBR_new = yBR - ROL * sin(phi);
				
				xBL_new = xBL - ROL * cos(phi);
				yBL_new = yBL - ROL * sin(phi);
				
				xFR_new = xFR - ROL * cos(phi);
				yFR_new = yFR - ROL * sin(phi);
				
				xFL_new = xFL - ROL * cos(phi);
				yFL_new = yFL - ROL * sin(phi);
			}
			break;	
		// right roll of the four wheels of ROL
		case(R_4):
			if(isRectangle)
			{
				moved = true;
				
				xBR_new = xBR + ROL * sin(phi);
				yBR_new = yBR - ROL * cos(phi);
				
				xBL_new = xBL + ROL * sin(phi);
				yBL_new = yBL - ROL * cos(phi);
				
				xFR_new = xFR + ROL * sin(phi);
				yFR_new = yFR - ROL * cos(phi);
				
				xFL_new = xFL + ROL * sin(phi);
				yFL_new = yFL - ROL * cos(phi);
			}
			break;
		// left roll of the four wheels of ROL	
		case(L_4):
			if(isRectangle)
			{
				moved = true;
				
				xBR_new = xBR - ROL * sin(phi);
				yBR_new = yBR + ROL * cos(phi);
				
				xBL_new = xBL - ROL * sin(phi);
				yBL_new = yBL + ROL * cos(phi);
				
				xFR_new = xFR - ROL * sin(phi);
				yFR_new = yFR + ROL * cos(phi);
				
				xFL_new = xFL - ROL * sin(phi);
				yFL_new = yFL + ROL * cos(phi);
			}
			break;	
			
		case(FW_S_BR):
			if(canStepBR && canStepBL)
			{
				moved = true;
				
				xBR_new = xBR + STEP * cos(phi);
				yBR_new = yBR + STEP * sin(phi);
				
				xBL_new = xBL;
				yBL_new = yBL;
				
				xFR_new = xFR;
				yFR_new = yFR;
				
				xFL_new = xFL;
				yFL_new = yFL;
			}
			else
			{
				if(canStepBR && !canStepBL)
				{
					moved = true;
					
					xBR_new = xBR + STEP * cos(phi);
					yBR_new = yBR + STEP * sin(phi);
					
					xBL_new = xBL;
					yBL_new = yBL;
					
					xFR_new = xFR + STEP * cos(phi);
					yFR_new = yFR + STEP * sin(phi);
					
					xFL_new = xFL + STEP * cos(phi);
					yFL_new = yFL + STEP * sin(phi);
				}
			}
			
			break;
			
		case(FW_S_BL):
			if(canStepBL && canStepBR)
			{
				moved = true;
				
				xBR_new = xBR;
				yBR_new = yBR;
				
				xBL_new = xBL + STEP * cos(phi);
				yBL_new = yBL + STEP * sin(phi);
				
				xFR_new = xFR;
				yFR_new = yFR;
				
				xFL_new = xFL;
				yFL_new = yFL;
			}
			else
			{
				if(canStepBL && !canStepBR)
				{
					moved = true;
					
					xBR_new = xBR;
					yBR_new = yBR;
					
					xBL_new = xBL + STEP * cos(phi);
					yBL_new = yBL + STEP * sin(phi);
					
					xFR_new = xFR + STEP * cos(phi);
					yFR_new = yFR + STEP * sin(phi);
					
					xFL_new = xFL + STEP * cos(phi);
					yFL_new = yFL + STEP * sin(phi);
				}
			}
			break;
			
			
		case(FW_S_FR):
			if(canStepFR && canStepFL)
			{
				moved = true;
				
				xBR_new = xBR + STEP * cos(phi);
				yBR_new = yBR + STEP * sin(phi);
				
				xBL_new = xBL + STEP * cos(phi);
				yBL_new = yBL + STEP * sin(phi);
				
				xFR_new = xFR + STEP * cos(phi);
				yFR_new = yFR + STEP * sin(phi);
				
				xFL_new = xFL;
				yFL_new = yFL;
			}
			else
			{
				if(canStepFR && !canStepFL)
				 {
					moved = true;
					
					xBR_new = xBR;
					yBR_new = yBR;
					
					xBL_new = xBL;
					yBL_new = yBL;
					
					xFR_new = xFR + STEP * cos(phi);
					yFR_new = yFR + STEP * sin(phi);
					
					xFL_new = xFL;
					yFL_new = yFL; 
			      }
			}
			break;
			
		case(FW_S_FL):
			if(canStepFL && canStepFR)
			{
				moved = true;
				
				xBR_new = xBR + STEP * cos(phi);
				yBR_new = yBR + STEP * sin(phi);
				
				xBL_new = xBL + STEP * cos(phi);
				yBL_new = yBL + STEP * sin(phi);
				
				xFR_new = xFR;
				yFR_new = yFR;
				
				xFL_new = xFL + STEP * cos(phi);
				yFL_new = yFL + STEP * sin(phi);
				
			}
			else{
				if(canStepFL && !canStepFR)
				{
					moved = true;
					
					xBR_new = xBR;
					yBR_new = yBR;
					
					xBL_new = xBL;
					yBL_new = yBL;
					
					xFR_new = xFR;
					yFR_new = yFR;
					
					xFL_new = xFL + STEP * cos(phi);
					yFL_new = yFL + STEP * sin(phi);
				}
			}
			break;
	}
	if (moved){
		r_result->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = xBR_new;
		r_result->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = yBR_new;
		r_result->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = xBL_new;
		r_result->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = yBL_new;
		r_result->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = xFR_new;
		r_result->as<ob::RealVectorStateSpace::StateType>(2)->values[1] = yFR_new;
		r_result->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = xFL_new;
		r_result->as<ob::RealVectorStateSpace::StateType>(3)->values[1] = yFL_new;
	}
}



// State validity function: returns true if the state sampled satisfies the bounds and if it is not inside a gap)
bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{   
	// Cast the start state to the desired type (consistent to the state space defined in plan())
    auto R2_4state = state->as<ob::CompoundStateSpace::StateType>();
    
    // Extract x,y position of the back right (B) and front right (F) foot and orientation of the body
    auto posBR = R2_4state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    double xBR = posBR[0], yBR = posBR[1];
    auto posBL = R2_4state->as<ob::RealVectorStateSpace::StateType>(1)->values;
    double xBL = posBL[0], yBL = posBL[1];
    auto posFR = R2_4state->as<ob::RealVectorStateSpace::StateType>(2)->values;
    double xFR = posFR[0], yFR = posFR[1];
    auto posFL = R2_4state->as<ob::RealVectorStateSpace::StateType>(3)->values;
    double xFL = posFL[0], yFL = posFL[1];
      
    
    double lengthR = sqrt(pow(posBR[0]-posFR[0],2) + pow(posBR[1]-posFR[1],2));
    double lengthL = sqrt(pow(posBL[0]-posFL[0],2) + pow(posBL[1]-posFL[1],2));
    double widthB = sqrt(pow(posBR[0]-posBL[0],2) + pow(posBR[1]-posBL[1],2));
    double widthF = sqrt(pow(posFR[0]-posFL[0],2) + pow(posFR[1]-posFL[1],2));
    
    bool check_bounds = si->satisfiesBounds(state);
    
    bool check_lenghtR = lengthR > L_MIN-TOL && lengthR < L_MAX + TOL;
    bool check_lenghtL = lengthL > L_MIN-TOL && lengthL < L_MAX + TOL;
    bool check_widthB = widthB > W_MIN-TOL && widthB < W_MAX + TOL;
    bool check_widthF = widthF > W_MIN-TOL && widthF < W_MAX + TOL;
    bool check_geometry = check_lenghtR && check_lenghtL && check_widthB && check_widthF;
    
    bool check_gapBR = xBR < X_GAP_MIN || xBR > X_GAP_MAX || yBR < Y_GAP_MIN || yBR > Y_GAP_MAX;
    bool check_gapBL = xBL < X_GAP_MIN || xBL > X_GAP_MAX || yBL < Y_GAP_MIN || yBL > Y_GAP_MAX;
    bool check_gapFR = xFR < X_GAP_MIN || xFR > X_GAP_MAX || yFR < Y_GAP_MIN || yFR > Y_GAP_MAX;
    bool check_gapFL = xFL < X_GAP_MIN || xFL > X_GAP_MAX || yFL < Y_GAP_MIN || yFL > Y_GAP_MAX;
    bool check_gap = check_gapBR && check_gapBL && check_gapFR && check_gapFL;
    
    
    bool check_obsXMIN = xBR < X_OBS_MIN && xBL < X_OBS_MIN && xFR < X_OBS_MIN && xFL < X_OBS_MIN;
    bool check_obsXMAX = xBR > X_OBS_MAX && xBL > X_OBS_MAX && xFR > X_OBS_MAX && xFL > X_OBS_MAX;
    bool check_obsYMIN = yBR < Y_OBS_MIN && yBL < Y_OBS_MIN && yFR < Y_OBS_MIN && yFL < Y_OBS_MIN;
    bool check_obsYMAX = yBR > Y_OBS_MAX && yBL > Y_OBS_MAX && yFR > Y_OBS_MAX && yFL > Y_OBS_MAX;
    bool check_obs = check_obsXMIN || check_obsXMAX || check_obsYMIN || check_obsYMAX;
    
    return check_bounds && check_geometry && check_gap && check_obs;
}


void plan()
{
	ob::PlannerStatus solved = ob::PlannerStatus::UNKNOWN;
	int n = 0;
	do {
		// Create the state space
		auto R2BR = std::make_shared<ob::RealVectorStateSpace>(2);
		auto R2BL = std::make_shared<ob::RealVectorStateSpace>(2);
		auto R2FR = std::make_shared<ob::RealVectorStateSpace>(2);
		auto R2FL = std::make_shared<ob::RealVectorStateSpace>(2);
		
		ob::RealVectorBounds bounds(2);
		bounds.setLow(0, -0.35);
		bounds.setLow(1, -1.65);
		bounds.setHigh(0, 3.35);
		bounds.setHigh(1, 0.35);
		
		R2BR->setBounds(bounds);
		R2BL->setBounds(bounds);
		R2FR->setBounds(bounds);
		R2FL->setBounds(bounds);
			
		
		std::vector<ob::StateSpacePtr> components {R2BR, R2BL, R2FR, R2FL};
		std::vector<double> weights {1.0, 1.0, 1.0, 1.0};
		auto space = std::make_shared<ob::CompoundStateSpace>(components, weights);
		
		// Create the control space
		auto cspace = std::make_shared<ompl::control::DiscreteControlSpace>(space, 0, 10);
		
		// Create an instance to the SpaceInformation
		auto si(std::make_shared<oc::SpaceInformation>(space, cspace));
		
		// In the propagation of the tree, a primitive can be applied only one at time 
		si-> setMinMaxControlDuration(1, 4);
		si->setPropagationStepSize(1.0);
		// Set the state validity checker function
		si->setStateValidityChecker(
			[&si](const ob::State *state) { return isStateValid(si.get(), state); });
	 
		// Set the propagator
		si->setStatePropagator(propagateWhileValid);

		// Create a start state
		
		ob::ScopedState<ob::CompoundStateSpace> start(space);
		// wheel_4
		double xBR_start = -0.35, yBR_start = -0.35;
		// wheel_3
		double xBL_start = -0.35, yBL_start = 0.35;
		// wheel_2
		double xFR_start = 0.35, yFR_start = -0.35;
		// wheel_1
		double xFL_start = 0.35, yFL_start = 0.35;
		
		start->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = xBR_start;
		start->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = yBR_start;
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = xBL_start;
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = yBL_start;
		start->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = xFR_start;
		start->as<ob::RealVectorStateSpace::StateType>(2)->values[1] = yFR_start;
		start->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = xFL_start;
		start->as<ob::RealVectorStateSpace::StateType>(3)->values[1] = yFL_start;
		

		// Create a goal state
		ob::ScopedState<ob::CompoundStateSpace> goal(space);
		
		double xBR_goal = 2.5, yBR_goal = -0.35;
		double xBL_goal = 2.5, yBL_goal = 0.35;
		double xFR_goal = 3.2, yFR_goal = -0.35;
		double xFL_goal = 3.2, yFL_goal = 0.35;
		
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = xBR_goal;
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = yBR_goal;
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = xBL_goal;
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = yBL_goal;
		goal->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = xFR_goal;
		goal->as<ob::RealVectorStateSpace::StateType>(2)->values[1] = yFR_goal;
		goal->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = xFL_goal;
		goal->as<ob::RealVectorStateSpace::StateType>(3)->values[1] = yFL_goal;
		 
		// Create a problem instance
		auto pdef(std::make_shared<ob::ProblemDefinition>(si));
	 
		// Cet the start and goal states
		pdef->setStartAndGoalStates(start, goal, THRESHOLD);
	 
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
