#include"Knapsack.h"
#include"LR.h"
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
	//ofstream fout;
	//fout.open("../experiment_result/10x100x050(All).txt");
	//vector<MultidimensionalKnapsack> problems = readInstances("../instances/OR10x100/OR10x100.dat");
	//fout << "n = 100, m = 10, a = 0,50" << endl << endl;
	//double avg = 0;
	vector<Item> items;
	items.push_back(Item{ 84,{20,41} });
	items.push_back({ 34,{12,51} });
	items.push_back({ 31,{7,24} });
	items.push_back({ 14,{75,40} });
	items.push_back({ 67,{93,84} });
	items.push_back({ 65,{21,70} });
	items.push_back({ 86,{75,34} });
	items.push_back({ 98,{67,41} });
	items.push_back({ 50,{34,49} });
	items.push_back({ 7,{28,27} });
	MultidimensionalKnapsack m = { 10,2,items,{190,250} };
	getLambda(m);
	int x = 5;
}