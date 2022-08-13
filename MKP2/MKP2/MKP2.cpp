#include"Knapsack.h"
#include"LR.h"
#include"BranchAndBound.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <omp.h>

using namespace std;

vector<MultidimensionalKnapsack> readInstances(string path) {
	ifstream fin(path);
	// Count of samples.
	int count;
	fin >> count;
	vector<MultidimensionalKnapsack> problems(count);
	for (size_t i = 0; i < count; i++)
	{
		int n, m, o;
		fin >> n >> m >> o;
		vector<Item> items(n);
		vector<int> b(m);
		for (size_t j = 0; j < n; j++)
		{
			int v;
			fin >> v;
			items[j].addValue(v);
			items[j].id = j;
		}
		for (size_t j = 0; j < n; j++)
		{
			for (size_t k = 0; k < m; k++)
			{
				int w;
				fin >> w;
				items[j].addConstraight(w);
			}
		}
		for (size_t j = 0; j < m; j++)
		{
			int x;
			fin >> x;
			b[j] = x;
		}
		problems[i] = MultidimensionalKnapsack(n, m, items, b);
	}
	return problems;
}

int main(int argc, char* argv[])
{
	vector<MultidimensionalKnapsack> problems = readInstances("../instances/OR5x100/OR5x100.dat");
	double x = BnBsolve(problems[0]);
	int y = 5;
	// 34675
}