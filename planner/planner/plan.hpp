#pragma once
#include "sim\dynamics.hpp"
#include <vector>
#include <random>
#include <map>

//using namespace std;
namespace plan
{ 

struct Action
{
	sim::ControllerTarget target;
	sim::ControllerParams params;
};


/*******************************************************************************
* StateNode declaration
******************************************************************************/
class ActionNode; //forward declaration

class StateNode
{
public:
	StateNode(StateNode* parent, sim::TimeState state, Action action, int depth) 
		: parent(parent), ts(state), a(action), depth(depth), children(0)
	{
		reward = eval(ts); //observe reward
	}

	Action a; 			//the action that led to this state
	
	StateNode* parent;  //The state node this node was sampled from

	double reward;		//Observed reward of this node's state

	sim::TimeState ts; 	//State represented by this node

	int depth; 			//Depth in the tree

	std::vector<ActionNode*> children; //Root actions sampled from this state
	
	//Reward function
	double eval(sim::TimeState state);

	void add_child(ActionNode* A);
};


/*******************************************************************************
* ActionNode declaration
******************************************************************************/
class ActionNode
{
public:
	ActionNode(StateNode* parent, Action root);

	StateNode* parent;

	Action root_action; //"Root" action from which discrepancies are generated

	//Random deviations from the root action
	std::vector<Action> discrepancies;
	std::map<int, StateNode*> discrepancy_map;

	double average_reward; 	//The average reward observed from discrepancies of this root action

	StateNode* generate_discrepancy();

	void add_discrepancy(Action a, StateNode* child);

	StateNode* get_statenode(Action a);

	static sim::Environment env;

private:
	//uniform random number generators
	std::uniform_real_distribution<> r_param;
	std::uniform_real_distribution<> r_epsilon;
};


/*******************************************************************************
* Planner declaration
******************************************************************************/
class Planner
{
public:
	Planner(sim::TimeState s0, int horizon, sim::Environment env);
	std::vector<Action> mcds(); //

private:
	StateNode* root;

	int max_depth; 	//Planning horizon

	double tau;    	//The threshhold for valid/invalid path

	Action default_action;

	std::vector<Action> plan(StateNode* v);

	StateNode* choose(StateNode* v);

	Action random_action();

	std::uniform_real_distribution<> URD; 	//uniform random number generator

};
} //namespace plan
