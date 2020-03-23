#ifndef MESH_H
#define MESH_H
#include <math.h>
#include <iostream>
#include <GL/glut.H>
#include <cstdlib> 
#include <vector>
#include <memory>
#include "Eigen/Dense.h"
#include "Eigen/Core"
#include "Eigen/SVD"
#include "Polygon.h"
using namespace std;
using Eigen::MatrixXd;
#define THRES 0.4	
#define MAX_GROW 10000
#define MAX_HEIGHT 0.8
#define MAX_WIDTH  0.75
#define MAX_DEPTH  0.75
#define MIN_HEIGHT 0.5
#define MIN_WIDTH  0.45
#define MIN_DEPTH  0.4
#define MAX_RAY    0.75
#define CURV_THRES 1
#define CURV_DIFF  2
#define FIND_PARTS 3
#define JOIN_PARTS 4

class Mesh {
public:
	// identification of the mesh
	int index;
	std::vector<float> color;

	// vertex number and vertex coordinates
	int verN, verN0;
	std::vector<std::shared_ptr<Vertex>> ver;
	std::vector<int> memV; // this vector storage the position in the memory of each point

	// number of triangles and the triangle vertex numbers
	int triN, triN0;
	std::vector<std::shared_ptr<Polygon>> tri;
	std::vector<int> memT;

	// average length of side (used for colour uneveness)
	float aveE;
	float thres;
	int edgeN; // number of edges

	// booleans
	bool invert_normal, upsidedown, is_tooth, bigger_than_limits, smaller_than_limits, is_gum;

	// var to avoid passing shared pointers 
	std::shared_ptr<Mesh> edit_mesh;

	// parameters of the mesh
	std::vector<float> max, min, mid, size;

	// parameters of the tooth
	float max_height, max_width, max_depth, min_height, min_width, min_depth, max_ray, lowest_value, highest_value;
	std::vector<float> axisX, axisY, axisZ;
	std::shared_ptr<Vertex> lowest_point, highest_point;

	// to avoid bugs
	int grow_count, grow_count2;

	/************ Prepare the Mesh *****************/
	Mesh(char* file_name, float max_h, float max_w, float max_d, float min_h, float min_w, float min_d, float max_r); // for the original group

	Mesh(std::shared_ptr<Mesh> other, int i); // constructor for the other subgroups

	void randomColor(); // randomly choose a color for the group

	std::shared_ptr<Vertex> randomPoint(); // randomly choose a point as seed for the segmentation 

	void normalizeSize(float length); // normalize so the mesh fit the screen

	/************ Manage Memory *****************/
	void reset(); // reset the group back to the original configuration

	void clearVectors(); // clear var

	void addVertex(std::shared_ptr<Vertex> v);

	void addPolygon(std::shared_ptr<Polygon> t);

	void delVertex(std::shared_ptr<Vertex> v);

	void delPolygon(std::shared_ptr<Polygon> t);

	void verticesTransfer(std::shared_ptr<Mesh> new_mesh, std::vector<std::shared_ptr<Vertex>> vertices);

	void facesTransfer(std::shared_ptr<Mesh> new_mesh, std::vector<std::shared_ptr<Polygon>> faces);

	void reconstruct(std::shared_ptr<Mesh> new_mesh);

	int countThreshold();

	/************ Modification and Calculation *****************/
	void computeNormals();

	void invertNormals();

	void computeLaplacians(); // 

	void findNeighbors();

	void smoothSurface(int nTimes);

	void computeCurvature();

	void setThreshold(int nTimes);

	void regionGrowing(std::shared_ptr<Vertex> seed);

	int regionGrowing2(std::shared_ptr<Vertex> seed);

	int regionGrowing3(std::shared_ptr<Vertex> seed, bool add);

	void regionGrowing4(std::shared_ptr<Vertex> seed);

	int segmentation(std::shared_ptr<Mesh> new_mesh, int mode);

	int linkPath(std::shared_ptr<Polygon> init, int end, int current, int maxValue, std::shared_ptr<Mesh> new_mesh);

	int fillHoles(std::shared_ptr<Mesh> new_mesh);

	void fitBox();

	/************ Differents Displays Functions *****************/
	void plotPoints(); // 

	void plotVector();
	
	void flatShading(); // 

	void showCurvature(); // 

	void showThreshold(); // 

	void solidColor();

	void showBorder();

};
#endif