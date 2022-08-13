#include"Knapsack.h"
#include"LR.h"
#include <limits>
#include <algorithm>
#include <vector>

using namespace std;

pair<vector<int>, double> knapsackDynamicSolve(MultidimensionalKnapsack kp, vector<int> I, vector<int> E)
{
	int n = kp.getN();
	vector<Item> items = kp.getItems();
	double c = 0;
	int b = kp.getB()[0];
	for (int i = 0; i < I.size(); i++) {
		c += items[I[i]].getValue();
		b -= items[I[i]].getConstraights()[0];
	}
	vector<vector<double>> dp(n - I.size() - E.size() + 1);
	for (size_t i = 0; i <= n - I.size() - E.size(); i++)
	{
		dp[i] = vector<double>(b + 1, 0);
	}
	int k = 0;
	vector<int> free(n);
	for (int i = 0; i <= n; i++) {
		if (std::find(I.begin(), I.end(), i) != I.end()) {
			k++;
			continue;
		}
		if (std::find(E.begin(), E.end(), i) != E.end()) {
			k++;
			continue;
		}
		for (int j = 0; j <= b; j++) {
			if (i - k == 0 || j == 0) {
				dp[i - k][j] = 0;
				continue;
			}
			if (j - items[i - 1].getConstraights()[0] >= 0) {
				dp[i - k][j] = max(dp[i - k - 1][j], dp[i - k - 1][j - items[i - 1].getConstraights()[0]] + items[i - 1].getValue());
			}
			else {
				dp[i - k][j] = dp[i - k - 1][j];
			}
		}
		if (i - k > 0) {
			free[i - k - 1] = i - 1;
		}
	}
	vector<int> used(n, 0);
	for (size_t i = 0; i < I.size(); i++)
	{
		used[I[i]] = 1;
	}
	int i = n - I.size() - E.size(), j = b;
	while (i > 0 && j > 0)
	{
		if (dp[i][j] == 0) {
			break;
		}
		if (dp[i][j] == dp[i - 1][j]) {
			i--;
		}
		else {
			i--;
			used[free[i]] = 1;
			j -= items[free[i]].getConstraights()[0];
		}
	}
	return make_pair(used, dp[n - I.size() - E.size()][b] + c);
}
pair<vector<double>, double> knapsackLPSolve(MultidimensionalKnapsack kp, vector<int> I, vector<int> E)
{
	int n = kp.getN();
	vector<Item> items = kp.getItems();
	vector<pair<double, int>> ratio;
	for (int j = 0; j < n; j++)
	{
		// Item quality calculation.
		double q = (double)items[j].getValue() / items[j].getConstraights()[0];
		ratio.push_back(make_pair(q, j));
	}
	sort(ratio.begin(), ratio.end());
	reverse(ratio.begin(), ratio.end());
	vector<int> ratio_order;
	for (int j = 0; j < n; j++)
	{
		ratio_order.push_back(ratio[j].second);
	}
	double c = 0;
	double weight = 0;
	double b = kp.getB()[0];
	vector<double> used(n, 0);
	for (int j = 0; j < I.size(); j++)
	{
		c += items[I[j]].getValue();
		weight += items[I[j]].getConstraights()[0];
		used[I[j]] = 1;
	}
	for (int j = 0; j < n; j++)
	{
		if (std::find(I.begin(), I.end(), ratio_order[j]) != I.end()) {
			continue;
		}
		if (std::find(E.begin(), E.end(), ratio_order[j]) != E.end()) {
			continue;
		}
		if (weight + items[ratio_order[j]].getConstraights()[0] <= b) {
			used[ratio_order[j]] = 1;
			weight += items[ratio_order[j]].getConstraights()[0];
			c += items[ratio_order[j]].getValue();
		}
		else {
			used[ratio_order[j]] = (b - weight) / items[ratio_order[j]].getConstraights()[0];
			c += items[ratio_order[j]].getValue() * (b - weight) / items[ratio_order[j]].getConstraights()[0];
			weight = b;
		}
		if (weight == b) {
			break;
		}
	}
	return make_pair(used, c);

}

pair<double, pair<int, vector<double>>> getLRBound(MultidimensionalKnapsack mkp, vector<int> I, vector<int> E, vector<double> lambda, int active)
{
	int m = mkp.getM();
	int n = mkp.getN();
	vector<int> b = mkp.getB();
	vector<Item> items = mkp.getItems();
	vector<double> best_used = vector<double>(n);
	vector<Item> temp_items(n);
	for (size_t k = 0; k < n; k++)
	{
		temp_items[k].id = k;
		temp_items[k].addConstraight(items[k].getConstraights()[active]);
		temp_items[k].addValue(items[k].getValue());
	}
	double pen = 0;
	int t_t = 0;
	for (size_t t = 0; t < m; t++)
	{
		if (t == active) {
			continue;
		}
		pen += lambda[t_t] * b[t];
		for (size_t z = 0; z < n; z++)
		{
			temp_items[z].addValue(temp_items[z].getValue() - items[z].getConstraights()[t] * lambda[t_t]);

		}
		t_t++;
	}
	MultidimensionalKnapsack temp = MultidimensionalKnapsack(n, 1, temp_items, vector<int>(1, b[active]));
	int bb = temp.getB()[0];
	for (int i = 0; i < I.size(); i++) {
		bb -= items[I[i]].getConstraights()[0];
	}
	if (bb < 0) {
		return make_pair(-1, make_pair(-1, best_used));
	}
	pair<vector<double>, double> res = knapsackLPSolve(temp, I, E);
	res.second += pen;
	double best = res.second;
	best_used = res.first;

	int branch_var = -1;
	int branch_max = -999999;
	for (size_t i = 0; i < n; i++)
	{
		if (std::find(I.begin(), I.end(), i) != I.end()) {
			continue;
		}
		if (std::find(E.begin(), E.end(), i) != E.end()) {
			continue;
		}
		double val = temp_items[i].getValue() / temp_items[i].getConstraights()[0];
		if (val > branch_max) {
			branch_var = i;
			branch_max = val;
		}
	}
	return make_pair(best, make_pair(branch_var, best_used));
}

pair<vector<double>, int> getLambda(MultidimensionalKnapsack mkp, vector<int> I, vector<int> E)
{
	int m = mkp.getM();
	int n = mkp.getN();
	vector<int> b = mkp.getB();
	vector<Item> items = mkp.getItems();
	vector<double> lambda(m - 1, 0);
	double bestFeasible = 0;
	int count = 0;
	double delta = 2;

	double best = numeric_limits<double>::max();
	int active = 0;
	for (size_t j = 0; j < m; j++)
	{
		vector<Item> temp_items(n);
		for (size_t k = 0; k < n; k++)
		{
			temp_items[k].addValue(items[k].getValue());
			temp_items[k].id = k;
			temp_items[k].addConstraight(items[k].getConstraights()[j]);
		}

		MultidimensionalKnapsack temp = MultidimensionalKnapsack(n, 1, temp_items, vector<int>(1, b[j]));
		pair<vector<double>, double> res = knapsackLPSolve(temp, I, E);
		if (res.second < best) {
			best = res.second;
			active = j;
		}
	}

	vector<double> best_used = vector<double>(n);
	for (size_t i = 0; i < 1000; i++)
	{
		double best = numeric_limits<double>::max();

		vector<Item> temp_items(n);
		for (size_t k = 0; k < n; k++)
		{
			temp_items[k].id = k;
			temp_items[k].addConstraight(items[k].getConstraights()[active]);
			temp_items[k].addValue(items[k].getValue());
		}
		double pen = 0;
		int t_t = 0;
		for (size_t t = 0; t < m; t++)
		{
			if (t == active) {
				continue;
			}
			pen += lambda[t_t] * b[t];
			for (size_t z = 0; z < n; z++)
			{
				temp_items[z].addValue(temp_items[z].getValue() - items[z].getConstraights()[t] * lambda[t_t]);

			}
			t_t++;
		}
		MultidimensionalKnapsack temp = MultidimensionalKnapsack(n, 1, temp_items, vector<int>(1, b[active]));
		pair<vector<double>, double> res = knapsackLPSolve(temp, I, E);
		res.second += pen;
		if (res.second < best) {
			best = res.second;
			best_used = res.first;
			count = 0;
		}
		else {
			count++;
		}
		if (count == 15) {
			delta /= 2;
			count = 0;
		}
		double stepsize = delta * (res.second - 34675);
		double sum = 0;
		vector<double> subgradient(m - 1);
		int j_s = 0;
		for (size_t j = 0; j < m; j++)
		{
			if (j == active) {
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
		for (size_t t = 0; t < m - 1; t++)
		{
			lambda[t] += stepsize * subgradient[t];
			if (lambda[t] < 0) {
				lambda[t] = 0;
			}
		}
	}
	return make_pair(lambda, active);
}