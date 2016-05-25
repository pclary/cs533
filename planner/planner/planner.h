#include "sim\dynamics.hpp"
#include <vector>
#include <random>

struct Action
{
	sim::ControllerParams params;
	sim::ControllerTarget target;
};

class StateNode
{
public:
	StateNode(StateNode* parent, sim::TimeState state, Action action)
		: parent(parent), children(std::vector<StateNode*>()), ts(state), val(0), action(action) {}

	StateNode* parent;
	std::vector<StateNode*> children;

	sim::TimeState ts;
	Action action;

	float val;
};

class Planner
{
	sim::Environment env;
	//TODO: clean up dynamic memory or conver to smart pointers
	StateNode* root;  
	StateNode* current_node;

	float tau;

	int max_depth;
	int depth;

	std::random_device rdev;
	std::default_random_engine reng;

	void branch_default();
	void branch_tree();

	void evaluate(StateNode* node);

public:
	Planner(sim::TimeState s0, int horizon);
	Action update();
};
