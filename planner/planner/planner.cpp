#include "planner.h"

Planner::Planner(sim::TimeState s0, int horizon)
	: max_depth(horizon), reng(rdev())
{
	root = new StateNode( nullptr, s0, {0,0,0} );

	current_node = root;
}

void Planner::branch_default()
{
	sim::TimeState child_state = sim::simulate_hopper(current_node->ts.state, 3.0, env, {1}, {}).back();

	StateNode* child = new StateNode(current_node, child_state, { 0,0,0 });

	current_node->children.push_back(child);
	current_node = child;
}

void Planner::branch_tree()
{
	std::bernoulli_distribution roll(.5);
	//placeholder generation
	Action rand_action = { rand() % 6 - 3, (rand() % 100) / 1000, rand() % 2000 - 1000 };

	//"dumb" branching policy
	while (roll(reng) && current_node->parent != nullptr)
		current_node = current_node->parent;

	sim::TimeState next_state = sim::simulate_hopper(current_node->ts.state, 3.0, env, rand_action.target, rand_action.params).back();
	StateNode* rand_node = new StateNode(current_node, next_state, rand_action);

	evaluate(current_node);
}

void Planner::evaluate(StateNode* node)
{
	if (node->ts.state.y < 0.5 * node->ts.state.l_eq && node->ts.state.dx < .1)
		node->val = 0;
	else
		node->val = 1;
}

Action Planner::update()
{
	if (current_node->val < tau)
	{
		if (depth < max_depth)
			branch_default();
		else
			branch_tree();
	}
	
	return	{ 0,0,0 }; //placeholder
}
