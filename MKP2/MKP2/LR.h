#ifndef LR_H
#define LR_H

#include"Knapsack.h"
#include <algorithm>
#include <set>

using namespace std;

MultidimensionalKnapsack getRelaxation(MultidimensionalKnapsack mkp, vector<double> lambda);
pair<vector<int>, double> knapsackDynamicSolve(MultidimensionalKnapsack kp);
vector<double> getLambda(MultidimensionalKnapsack mkp);
bool isFeasible(MultidimensionalKnapsack knapsack, vector<int> used);

#endif