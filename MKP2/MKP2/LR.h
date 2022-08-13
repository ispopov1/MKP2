#ifndef LR_H
#define LR_H

#include"Knapsack.h"
#include <algorithm>
#include <set>

using namespace std;

pair<vector<int>, double> knapsackDynamicSolve(MultidimensionalKnapsack kp, vector<int> I, vector<int> E);
pair<vector<double>, double> knapsackLPSolve(MultidimensionalKnapsack kp, vector<int> I, vector<int> E);
pair<double, pair<int, vector<double>>> getLRBound(MultidimensionalKnapsack mkp, vector<int> I, vector<int> E, vector<double> lambda, int active);
pair<vector<double>, int> getLambda(MultidimensionalKnapsack mkp, vector<int> I, vector<int> E);


#endif