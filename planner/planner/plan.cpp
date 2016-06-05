#include "plan.hpp"
#include <iterator>
#include <algorithm>

//#define VIS_DEBUG
#ifdef VIS_DEBUG
	#include <thread>
	#include "vis\Hopper.hpp"
	//Debug info/visualization macro
	#define VISUALIZE(ss, parent, env, a, tag) do {								\
		std::cout << "----------------------------" << std::endl;				\
		std::cout << tag << ", depth: " << parent->depth + 1 << std::endl;		\
		std::cout << " target: " << a.target.velocity << std::endl;				\
		std::cout << " horiz leg: " << a.params.horizontal_push << std::endl;   \
		std::cout << " leg exten: " << a.params.leg_extension << std::endl;		\
		std::cout << " x: " << ss.back().state.x << std::endl;					\
		std::cout << "----------------------------" << std::endl;				\
		vis::Hopper hopper(env);												\
		hopper.animate(ss);														\
		this_thread::sleep_for(chrono::milliseconds(0));						\
	} while (0)

#endif

namespace plan
{ 

/*******************************************************************************
* Private namespace variables/ static variable initialization
******************************************************************************/

std::random_device rdev;
std::default_random_engine eng(rdev());

//static variable
sim::Environment ActionNode::env;

/*******************************************************************************
* Utility functions
******************************************************************************/

//scales val by rand%
#define DELTA(val, rand) val + val * rand

inline int argmax(std::map<int, StateNode*> map)
{
	int i_max;
	double max_reward = -INFINITY;
	for (auto const &pair : map)
	{
		if (pair.second->reward > max_reward)
		{
			max_reward = pair.second->reward;
			i_max = pair.first;
		}
	}
	return i_max;
}

inline bool operator==(const Action& lhs, const Action& rhs)
{
	return (lhs.target.velocity == rhs.target.velocity &&
		lhs.params.horizontal_push == rhs.params.horizontal_push &&
		lhs.params.leg_extension == lhs.params.leg_extension);
}


/*******************************************************************************
* StateNode functions
******************************************************************************/

//Reward function
double StateNode::eval(sim::TimeState state)
{
	//upright states are better, must be moving forward
	double height_factor = state.state.y / state.state.l_eq;
	int velocity_factor = (state.state.dx > .1 ? 1 : 0);

	int sign = height_factor >= 0 ? 1 : -1;
	
	//squared to prioritize upright states
	return  height_factor * height_factor * sign * velocity_factor;
}

void StateNode::add_child(ActionNode * A)
{
	children.push_back(A);
}


/*******************************************************************************
* ActionNode functions and constructor
******************************************************************************/

ActionNode::ActionNode(StateNode * parent, Action root)
	: parent(parent), root_action(root), r_epsilon(0, 1), r_param(-.2, .2) //20% radius around parameter value
{
	//add (root action, statenode) pair to hash table
	sim::StateSeries ss = sim::simulate_hopper(parent->ts.state, 3.0, env, root_action.target, root_action.params);
	sim::TimeState s = ss.back();

#ifdef VIS_DEBUG 	
	VISUALIZE(ss, parent, env, root_action, "NORMAL");
#endif

	StateNode* child = new StateNode(parent, s, root_action, parent->depth + 1);
	add_discrepancy(root_action, child);
}

StateNode* ActionNode::generate_discrepancy()
{
	//using epsilon greedy for ease of implementation
	//could definitely be changed to UCT or similar
	if(r_epsilon(eng) > .5) //explore
		return discrepancy_map[rand() % discrepancies.size()];
	

	else //exploit
	{
		if (r_epsilon(eng) > .5) 	//(discrepancy from best discrepancy)
		{
			//This could be made arbitrarily more complex/more heuristic
			//Could probably store argmax online instead of calculating it here
			Action amax = discrepancies[argmax(discrepancy_map)];

			Action a = { DELTA(amax.target.velocity, r_param(eng)),
						{DELTA(amax.params.leg_extension, r_param(eng)),
						 DELTA(amax.params.leg_extension, r_param(eng))} };

			sim::StateSeries ss = sim::simulate_hopper(parent->ts.state, 3.0, env, a.target, a.params);
			sim::TimeState s = ss.back();

#ifdef VIS_DEBUG 
			VISUALIZE(ss, parent, env, a, "DISCREPANCY2");
#endif

			StateNode* child = new StateNode(parent, s, a, parent->depth + 1);
			add_discrepancy(a, child);

			return child;
		}

		else //random discrepancy
		{
			Action a = { DELTA(root_action.target.velocity, r_param(eng)),
						{DELTA(root_action.params.leg_extension, r_param(eng)),
						 DELTA(root_action.params.leg_extension, r_param(eng))} };

			sim::StateSeries ss = sim::simulate_hopper(parent->ts.state, 3.0, env, a.target, a.params);
			sim::TimeState s = ss.back();

#ifdef VIS_DEBUG
			VISUALIZE(ss, parent, env, a, "DISCREPANCY1");
#endif

			StateNode* child = new StateNode(parent, s, a, parent->depth + 1);
			add_discrepancy(a, child);

			return child;
		}
	}
}

void ActionNode::add_discrepancy(Action a, StateNode* child)
{
	discrepancies.push_back(a);
	discrepancy_map[discrepancies.size() - 1] = child;
}

StateNode * ActionNode::get_statenode(Action a)
{
	int index = find(discrepancies.begin(), discrepancies.end(), a) - discrepancies.begin();
	return discrepancy_map[index];
}

/*******************************************************************************
* Planner functions and constructor
******************************************************************************/

Planner::Planner(sim::TimeState s0, int horizon, sim::Environment env)
	: max_depth(horizon), URD(0, 1)
{
	root = new StateNode(nullptr, s0, { {} }, 0);

	default_action = { 1,{} };

	tau = .89;

	ActionNode::env = env;
}


std::vector<Action> Planner::mcds()
{
	StateNode* v;
	do
	{
		v = root;
		while (v->depth < max_depth)
		{
			v = choose(v);

			if (v->reward < tau)//save computation
				break;
		}
	//Ends when it's found a valid path
	//Could be modified to continue looking for more robust paths
	} while (v->reward < tau);

	return plan(v);
}

//Extracts plan from terminal node
std::vector<Action> Planner::plan(StateNode * v)
{
	std::vector<Action> plan;
	do
	{
		plan.push_back(v->a);
		v = v->parent;
	} while (v->parent != nullptr);

	std::reverse(plan.begin(), plan.end());
	return plan;
}

inline Action Planner::random_action()
{
	return{ URD(eng) * 6 - 3, {URD(eng) / 10, URD(eng) * 2000 - 1000} };
}

StateNode* Planner::choose(StateNode * v)
{
	//If no children, do default policy
	if (v->children.size() == 0)
	{
		ActionNode* default_child = new ActionNode(v, default_action);
		v->add_child(default_child);

		return default_child->get_statenode(default_action);
	}

	//Otherwise use epsilon-greedy to explore/exploit action-nodes
	//Might be useful to swith to a different bandit algorithm
	else
	{
		if (URD(eng) > .4) //explore
		{
			Action a = random_action();
			ActionNode* explore_child = new ActionNode(v, a);
			v->add_child(explore_child);

			return explore_child->get_statenode(a);
		}

		else //exploit
		{
			//exploits a random node
			ActionNode* exploit_child = v->children[rand() % v->children.size()];

			StateNode* ecs = exploit_child->generate_discrepancy();

			return ecs;
		}
	}
}
}//namespace plan