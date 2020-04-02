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

namespace ob = ompl::base;
namespace oc = ompl::control;

// At each iteration:
// 1) The planner sample a state in order to expand the tree;
// 2) It finds the nearest available vertex from previous iterations
// 3) Try to connect the two vertex using a control action
// 4) Continues until the goal region is reached

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
    
    // Extract x,y position from SE2 state
    auto pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
    auto rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1)->value;
    
    // Cast the control to the desired type (consistent to the control space defined in plan())
    auto ctrl = control->as<oc::DiscreteControlSpace::ControlType>()->value;

    // Cast the result state to the desired type (consistent to the state space defined in plan())
    auto r_result = result->as<ob::SE2StateSpace::StateType>();
    
    // Pick a control and propagate the state (x and y will be only propagated)
    switch (ctrl)
    {
        case 0:
            r_result->setXY(pos[0] + 0.01, pos[1]);
            r_result->setYaw(rot);
            break;
        case 1:
            r_result->setXY(pos[0] - 0.01, pos[1]);
            r_result->setYaw(rot);
            break;
        case 2:
            r_result->setXY(pos[0], pos[1] + 0.01);
            r_result->setYaw(rot);
            break;
        case 3:
            r_result->setXY(pos[0], pos[1] - 0.01);
            r_result->setYaw(rot);
            break;
    }        
}

// State validity function: returns true if the state sampled satisfies the bounds and if it is not inside a rectangular obstacle)
bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    // Cast the state to the desired value and extract the position
    auto se2state = state->as<ob::SE2StateSpace::StateType>();
    const auto pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;

    // Check for state validity
    return si->satisfiesBounds(state) && (!(pos[0] > 1 && pos[0] < 2 && pos[1] < 2 && pos[1] > -2));
}

void plan()
{
    // Create the state space
    auto space = std::make_shared<ompl::base::SE2StateSpace>();
    
    // Create bounds and set them
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -0.5);
    bounds.setLow(1, -3.0);
    bounds.setHigh(0, 5.5);
    bounds.setHigh(1, 3.0);
    space->setBounds(bounds);
    
    // Create the control space
    auto cspace = std::make_shared<ompl::control::DiscreteControlSpace>(space, 0, 3);
    
    // Create an instance to the SpaceInformation
    auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

    // Set the state validity checker function
    si->setStateValidityChecker(
        [&si](const ob::State *state) { return isStateValid(si.get(), state); });
 
    // Set the propagator
    si->setStatePropagator(propagate);
 
    // Create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(0.0);
    start->setY(0.0);
    start->setYaw(0.0);
 
    // Create a goal state
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(5.0);
    goal->setY(1.5);
    goal->setYaw(0.0);
     
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
 
    // Attempt to solve within 5 sec (can be changed)
    ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);
 
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ob::PathPtr path = pdef->getSolutionPath();
        path->print(std::cout);
    } 
    else
        std::cout << "No solution found" << std::endl;
}

int main()
{
    plan();
    return 0;
}