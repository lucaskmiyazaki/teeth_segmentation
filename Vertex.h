#ifndef VERTEX_H
#define VERTEX_H

#include <math.h>
#include <iostream>
#include <GL/glut.H>
#include <cstdlib> 
#include <vector>
#include <memory>
using namespace std;


class Vertex {
public:
	int index;
	std::vector<float> pos;
	std::vector<float> origPos;
	std::vector<float> nor;
	std::vector<float> lap;
	std::vector<std::shared_ptr<Vertex>> nei;
	float curv;
	int neiN, neiAdded, lapN, triAdded;
	bool edge, analysed; // if vertex has a value higher then threshold 
	int group;

	Vertex(int i);

	float getPos(int n);

	void computeLaplacian();

	void curvatureComputation(float aveE);

	void back2original();

	void clearVector();

	void checkThreshold(float threshold);

	void smooth(float dt);

	void addNeighbor(std::shared_ptr<Vertex> n);

};
#endif