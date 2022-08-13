#include"Knapsack.h"
#include"LR.h"
#include"BranchAndBound.h"

using namespace std;

bool Node::operator<(const Node& rhs) const
{
	if (bound == rhs.bound) {
		return nodeID < rhs.nodeID;
	}
	return bound > rhs.bound;
}
bool Node::operator==(const Node& rhs) const
{
	return nodeID == rhs.nodeID;
}
bool Node::operator!=(const Node& rhs) const
{
	return nodeID != rhs.nodeID;
}


double BnBsolve(MultidimensionalKnapsack knapsack) {
	// Knapsack capacity
	int n = knapsack.getN();
	int m = knapsack.getM();
	vector<Item> items = knapsack.getItems();
	vector<int> b = knapsack.getB();
	// I - set of items guaranteed used in current node, E - guaranteed not used items.
	vector<int> I, E;
	double best = 0;
	vector<double> best_used(n);
	vector<Node> availableNodes;
	Node startNode;
	startNode.E = E;
	startNode.I = I;
	startNode.nodeID = 0;
	availableNodes.push_back(startNode);
	pair<vector<double>, int> lambda = getLambda(knapsack, startNode.I, startNode.E);
	while (availableNodes.size() > 0) {
		Node node = availableNodes[0];
		pair<double, pair<int, vector<double>>> x = getLRBound(knapsack, node.I, node.E, vector<double>(m-1,0), 0);
		node.bound = x.first;
		node.branchNum = x.second.first;
		cout << x.first << " " << node.I.size() << " " << node.E.size() << endl;
		if (isFeasible(knapsack, x.second.second)) {
			if (x.first > best) {
				best = x.first;
				best_used = x.second.second;
			}
			availableNodes.erase(availableNodes.begin());
			continue;
		}
		else {
			if (x.first < best || x.first < 34675 || node.I.size() + node.E.size() >= n) {
				availableNodes.erase(availableNodes.begin());
				continue;
			}
		}
		// Branching condition.
		if (node.branchNum != -1)
		{
			vector<int> I, E;
			Node left_node, right_node;
			I = node.I;
			E = node.E;
			// branch item number
			int num = node.branchNum;
			// Suppose branching item not included.
			E.push_back(num);
			left_node.I = I;
			left_node.E = E;
			left_node.nodeID = node.nodeID * 2 + 1;
			E.pop_back();
			I.push_back(num);
			right_node.I = I;
			right_node.E = E;
			right_node.nodeID = node.nodeID * 2 + 2;
			availableNodes.erase(availableNodes.begin());
			availableNodes.push_back(left_node);
			if (isFeasibleNode(knapsack, right_node.I)) {
				availableNodes.push_back(right_node);
			}
		}
	}

	return best;
}

bool isFeasible(MultidimensionalKnapsack knapsack, vector<double> used)
{
	int n = knapsack.getN();
	int m = knapsack.getM();
	vector<Item> items = knapsack.getItems();
	vector<int> b = knapsack.getB();
	bool feasible = true;
	double sum = 0;
	for (int j = 0; j < n; j++)
	{
		if ((used[j] > 0) && (used[j] < 1)) {
			return false;
		}
		if (used[j] == 1) {
			sum += items[j].getValue();
		}
	}
	for (int i = 0; i < m; i++)
	{
		double weight = 0;
		for (int j = 0; j < n; j++)
		{
			if (used[j] == 1) {
				weight += items[j].getConstraights()[i];
			}
		}
		if (weight > b[i]) {
			feasible = false;
			break;
		}
	}
	return feasible;
}
bool isFeasibleNode(MultidimensionalKnapsack knapsack, vector<int> I)
{
	int n = knapsack.getN();
	int m = knapsack.getM();
	vector<Item> items = knapsack.getItems();
	vector<int> b = knapsack.getB();
	bool feasible = true;
	double sum = 0;
	for (int i = 0; i < m; i++)
	{
		double weight = 0;
		for (int j = 0; j < I.size(); j++)
		{
			weight += items[I[j]].getConstraights()[i];
		}
		if (weight > b[i]) {
			feasible = false;
			break;
		}
	}
	return feasible;
}
