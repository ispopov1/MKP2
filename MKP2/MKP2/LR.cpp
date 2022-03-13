#include"Knapsack.h"
#include"LR.h"
#include <limits>
#include <algorithm>
#include <vector>

using namespace std;

double getRelaxationBound(MultidimensionalKnapsack mkp, vector<double> lambda)
{
	return 0;
}

pair<vector<int>, double> knapsackDynamicSolve(MultidimensionalKnapsack kp)
{
	int n = kp.getN();
	vector<Item> items = kp.getItems();
	int b = kp.getB()[0];
	vector<vector<int>> dp(n + 1);
	for (size_t i = 0; i < n + 1; i++)
	{
		dp[i] = vector<int>(b + 1, 0);
	}
	for (int i = 0; i <= n; i++) {
		for (int j = 0; j <= b; j++) {
			if (i == 0 || j == 0) {
				dp[i][j] = 0;
				continue;
			}
			if (j - items[i - 1].getConstraights()[0] >= 0) {
				dp[i][j] = max(dp[i - 1][j], dp[i - 1][j - items[i - 1].getConstraights()[0]] + items[i - 1].getValue());
			}
			else {
				dp[i][j] = dp[i - 1][j];
			}
		}
	}
	vector<int> used(n, 0);
	int i = n, j = b;
	while (i > 0 && j > 0)
	{
		if (dp[i][j] == 0) {
			break;
		}
		if (dp[i][j] == dp[i - 1][j]) {
			i--;
		}
		else {
			used[i - 1] = 1;
			i--;
			j -= items[i].getConstraights()[0];
		}
	}
	return make_pair(used, dp[n][b]);
}

vector<double> getLambda(MultidimensionalKnapsack mkp)
{
	int m = mkp.getM();
	int n = mkp.getN();
	vector<int> b = mkp.getB();
	vector<Item> items = mkp.getItems();
	vector<double> lambda(m - 1, 0);
	double bestFeasible = 0;
	double bound = numeric_limits<double>::max();
	int count = 0;
	double delta = 2;
	for (size_t i = 0; i < 500; i++)
	{
		for (size_t t = 0; t < m - 1; t++)
		{
			cout << lambda[t] << " ";
		}
		double best = numeric_limits<double>::max();
		int k = -1;
		vector<int> best_used = vector<int>(n);
		for (size_t j = 0; j < m; j++)
		{
			vector<Item> temp_items = mkp.getItems();
			double pen = 0;
			int t_t = 0;
			for (size_t t = 0; t < m; t++)
			{
				if (t == j) {
					continue;
				}
				pen += lambda[t_t] * b[t];
				for (size_t z = 0; z < n; z++)
				{
					temp_items[z].addValue(temp_items[z].getValue() - temp_items[z].getConstraights()[t] * lambda[t_t]);

				}
				t_t++;
			}
			MultidimensionalKnapsack temp = MultidimensionalKnapsack(n, 1, temp_items, vector<int>(1, b[j]));
			pair<vector<int>, double> res = knapsackDynamicSolve(temp);
			res.second += pen;
			if (res.second < best) {
				best = res.second;
				best_used = res.first;
				k = j;
			}
		}
		if (best < bound) {
			bound = best;
			count = 0;
		}
		else {
			count++;
		}
		if (count == 15) {
			delta /= 2;
			count = 0;
		}
		cout << bound << endl;
		double stepsize = delta * bound;
		double sum = 0;
		vector<double> subgradient(m - 1);
		int j_s = 0;
		for (size_t j = 0; j < m; j++)
		{
			if (j == k) {
				continue;
			}
			double add = -b[j];
			for (size_t t = 0; t < n; t++)
			{
				add += items[t].getConstraights()[j] * items[t].getValue() * best_used[t];
			}
			sum += add * add;
			subgradient[j_s] = add;
			j_s++;
		}
		stepsize /= sum;
		for (size_t t = 0; t < m-1; t++)
		{
			lambda[t] += stepsize*subgradient[t];
		}
	}
	return lambda;
}

bool isFeasible(MultidimensionalKnapsack knapsack, vector<int> used)
{
	int n = knapsack.getN();
	int m = knapsack.getM();
	vector<Item> items = knapsack.getItems();
	vector<int> b = knapsack.getB();
	vector<double> minItems(n);
	bool feasible = true;
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
