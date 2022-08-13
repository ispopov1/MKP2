#ifndef BRANCHANDBOUND_H
#define BRANCHANDBOUND_H

#include"Knapsack.h"
#include <algorithm>
#include <set>
#include <stack>

using namespace std;

class Node {
public:
	double bound;
	int branchNum;
	long long nodeID;
	vector<int> I, E;

	bool operator<(const Node& rhs) const;
	bool operator==(const Node& rhs) const;
	bool operator!=(const Node& rhs) const;
};

double BnBsolve(MultidimensionalKnapsack knapsack);
bool isFeasible(MultidimensionalKnapsack knapsack, vector<double> I);
bool isFeasibleNode(MultidimensionalKnapsack knapsack, vector<int> I);
#endif