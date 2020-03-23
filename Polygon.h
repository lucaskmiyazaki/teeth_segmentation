#ifndef POLYGON_H
#define POLYGON_H

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
	std::vector<std::shared_ptr<Vertex>> nei, origNei;
	std::vector<int> tri;
	float curv;
	int neiN, lapN, triN, nei2N, origNeiN;
	bool edge, analysed, border; // if vertex has a value higher then threshold 
	int group;

	Vertex(int i);

	float getPos(int n);

	void computeLaplacian();

	void curvatureComputation(float aveE);

	void back2original();

	void clearVector();

	void checkThreshold(float threshold);

	bool checkCurvDiff(std::shared_ptr<Vertex> n, float threshold);

	void smooth(float dt);

	void addNeighbor(std::shared_ptr<Vertex> n);

	void addFace(int t);

	void checkBorder();

	int surroundingGroup();

	void plot();

};

class Polygon {
public:
	std::vector<std::shared_ptr<Vertex>> origVer;
	std::vector<std::shared_ptr<Vertex>> ver;
	std::vector<float> nor;
	bool border, old;
	int index;
	int group;
	int neiN, nei2N; // number of added nei, total number of nei, total number of nei that shares the same 2 vertex
	std::vector<std::shared_ptr<Polygon>> nei, nei2;

	Polygon(int i);

	void prepVerticesLaplacian();

	void back2original();

	void clearVector();

	void addNeighbor(std::shared_ptr<Polygon> n);

	void checkBorder();

};


#endif