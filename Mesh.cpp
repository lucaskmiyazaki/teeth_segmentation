#include "Mesh.h"

/************ Prepare the Mesh *****************/
Mesh::Mesh(char* file_name, float max_h, float max_w, float max_d, float min_h, float min_w, float min_d, float max_r) {
	/* Read the mesh file */
	FILE* in = fopen(file_name, "r");
	index = 0;
	invert_normal = false;
	upsidedown = false;
	max_height = max_h;
	max_width  = max_w;
	max_depth  = max_d;
	min_height = min_h;
	min_width  = min_w;
	min_depth  = min_d;
	max_ray    = max_r;

	// read the number of vertices and faces
	int check1 = fscanf(in, "%d", &verN);
	int check2 = fscanf(in, "%d", &triN);
	if (check1 == 1 && check2 == 1)
	{
		printf("vertices: %d \n", verN);
		printf("faces: %d \n", triN);
		verN0 = verN;
		triN0 = triN;
	}
	else
	{
		printf("error: failed to read file \n");
	}

	// create smart pointers and vectors
	ver = std::vector<std::shared_ptr<Vertex>>(verN);
	memV = std::vector<int>(verN, -1);
	for (int i = 0; i < verN; i++)
	{
		ver[i] = std::make_shared<Vertex>(i);
		memV[i] = i;
	}
	tri = std::vector<std::shared_ptr<Polygon>>(triN);
	memT = std::vector<int>(triN, -1);
	for (int i = 0; i < triN; i++)
	{
		tri[i] = std::make_shared<Polygon>(i);
		memT[i] = i;
	}

	// read the info of vertices and faces
	for (int i = 0; i < verN; i++) {
		int check3 = fscanf(in, "%f %f %f", &ver[i]->origPos[0], &ver[i]->origPos[1], &ver[i]->origPos[2]);
		if (check3 != 3)
		{
			printf("error: failed to read file\n");
		}
	}

	for (int i = 0; i < triN; i++) {
		std::vector<int> temp;
		temp = std::vector<int>(3, 0);
		int check3 = fscanf(in, "%d %d %d", &temp[0], &temp[1], &temp[2]);
		if (check3 != 3)
		{
			printf("failed to read file");
		}
		else
		{
			tri[i]->origVer[0] = ver[temp[0]];
			tri[i]->origVer[1] = ver[temp[1]];
			tri[i]->origVer[2] = ver[temp[2]];
		}
	}

	aveE = 0;
	normalizeSize(4); 
	reset(); // clean the memory and set the points position to the original position
	randomColor();
	fclose(in);

	// find the lowest point and the highest
	lowest_point = ver[0];
	lowest_value = ver[0]->pos[2];
	for (int i = 1; i < verN; i++)
	{
		if (lowest_value > ver[i]->pos[2])
		{
			lowest_point = ver[i];
			lowest_value = ver[i]->pos[2];
		}
	}
	highest_point = ver[0];
	highest_value = ver[0]->pos[2];
	for (int i = 1; i < verN; i++)
	{
		if (highest_value < ver[i]->pos[2])
		{
			highest_point = ver[i];
			highest_value = ver[i]->pos[2];
		}
	}
}

Mesh::Mesh(std::shared_ptr<Mesh> other, int i) {  // cria um mesh vazio
	index = i;
	ver = std::vector<std::shared_ptr<Vertex>>(other->verN0);
	tri = std::vector<std::shared_ptr<Polygon>>(other->triN0);
	memV = std::vector<int>(other->verN0, -1);
	memT = std::vector<int>(other->triN0, -1);
	verN = 0;
	triN = 0;
	verN0 = other->verN0;
	triN0 = other->triN0;
	aveE = other->aveE;
	invert_normal = other->invert_normal;
	upsidedown = other->upsidedown;
	randomColor();
	max_height = other->max_height;
	max_width  = other->max_width;
	max_depth  = other->max_depth;
	min_height = other->min_height;
	min_width  = other->min_width;
	min_depth  = other->min_depth;
	max_ray    = other->max_ray;
	highest_point = other->highest_point;
	highest_value = other->highest_value;
	lowest_point = other->lowest_point;
	lowest_value = other->lowest_value;
}

void Mesh::randomColor() { // set a new color to the group
	color = std::vector<float>(3);
	color[0] = float(rand() % 1000) / 1000;
	color[1] = float(rand() % 1000) / 1000;
	color[2] = float(rand() % 1000) / 1000;
}

void Mesh::normalizeSize(float length) {// Create a box that fit the object, normalize this box
	float maxTemp[3], minTemp[3];
	float sizeTemp[3], midTemp[3];
	for (int i = 0; i < 3; i++) { // find max and min values
		maxTemp[i] = ver[0]->origPos[i];
		minTemp[i] = ver[0]->origPos[i];
		for (int j = 1; j < verN; j++) {
			float v = ver[j]->origPos[i];
			if (v > maxTemp[i])
				maxTemp[i] = v;
			else if (v < minTemp[i])
				minTemp[i] = v;
		}
		sizeTemp[i] = maxTemp[i] - minTemp[i];
		midTemp[i] = 0.5f * (maxTemp[i] + minTemp[i]);
	}

	// biggest dimension of the box is 4
	float s = sizeTemp[0];
	if (sizeTemp[1] > s)
		s = sizeTemp[1];
	if (sizeTemp[2] > s)
		s = sizeTemp[2];
	float scale = length / s;

	// normalize size and translate the centroid
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < verN; j++)
			ver[j]->origPos[i] = scale * (ver[j]->origPos[i] - midTemp[i]);

	// rotate to align the teeth to the axis
	MatrixXd m(3, verN);
	for (int i = 0; i < verN; i++)
	{
		for (int j = 0; j < 3; j++) {
			m(j, i) = ver[i]->origPos[j];
		}
	}
	Eigen::JacobiSVD<MatrixXd> svd; // apply singular value decomposition for the linear regression
	svd.compute(m, Eigen::ComputeThinU);
	MatrixXd u(3, 3);
	u = svd.matrixU();
	MatrixXd T(3, 3); // it is known that the transform matrix is the inverse of the U matrix
	T = u.inverse();
	for (int i = 0; i < verN; i++)
	{
		MatrixXd temp(3, 1);
		temp(0, 0) = ver[i]->origPos[0];
		temp(1, 0) = ver[i]->origPos[1];
		temp(2, 0) = ver[i]->origPos[2];
		temp = T * temp;
		ver[i]->origPos[0] = temp(0, 0);
		ver[i]->origPos[1] = temp(1, 0);
		ver[i]->origPos[2] = temp(2, 0);
	}

	// calculate the average distance between neighbors
	double total = 0;
	for (int i = 0; i < triN; i++) {
		std::vector<std::shared_ptr<Vertex>> v = tri[i]->origVer;
		for (int j = 0; j < 3; j++) {
			std::vector<float> p = v[j]->origPos;
			std::vector<float> q = v[(j + 1) % 3]->origPos;
			total += sqrt((q[0] - p[0]) * (q[0] - p[0]) + (q[1] - p[1]) * (q[1] - p[1]) + (q[2] - p[2]) * (q[2] - p[2]));
		}
	}
	aveE = total / (3 * triN);	
}

/************ Manage Memory *****************/
void Mesh::reset() {
	for (int i = 0; i < verN; i++)
	{
		ver[i]->back2original(); // arrumar aqui, precisa tirar o efeito do smooth tbm
		if (memV[ver[i]->index] != i)
		{
			printf("error: point does not belong in this position\n");
		}
		if (ver[i]->group != index) // aqui nao ha consistencia, reveja o codigo de alocacao dos pontos
		{
			printf("error: point does not belong in this mesh\n");
		}
	}
	for (int i = 0; i < triN; i++)
	{
		tri[i]->back2original();
		if (memT[tri[i]->index] != i)
		{
			printf("error: face does not belong in this position\n");
		}
		if (tri[i]->group != index)
		{
			printf("error: face does not belong in this mesh\n");
		}
	}
	clearVectors();
	findNeighbors(); 
	thres = THRES;
}

void Mesh::clearVectors() { // recalculate all normals and laplacians
	for (int i = 0; i < verN; i++)
	{
		ver[i]->clearVector();
	}
	for (int i = 0; i < triN; i++)
	{
		tri[i]->clearVector();
	}
	computeNormals();
	if (invert_normal)
	{
		invertNormals();
	}
	computeLaplacians();
}

void Mesh::addVertex(std::shared_ptr<Vertex> v) {
	if (verN-1 >= verN0 || v->index >= verN0)
	{
		printf("out of range\n");
	}
	if (memV[v->index] == -1) // if this one still doesnt belongs
	{
		ver[verN++] = v;
		v->group = index;
		memV[v->index] = verN - 1;
	}
	else if (v->group != index)
	{
		v->group = index;
	}
}

void Mesh::addPolygon(std::shared_ptr<Polygon> t) {
	if (memT[t->index] == -1) // if this one still doesnt belongs
	{
		tri[triN++] = t;
		t->group = index;
		memT[t->index] = triN - 1;
	}
	else if (t->group != index)
	{
		t->group = index;
	}
}

void Mesh::delVertex(std::shared_ptr<Vertex> v) {
	int memory = memV[v->index];
	if (memory >= 0)
	{
		std::shared_ptr<Vertex> last = ver[--verN]; // reallocate the lats vertex to the position of the deleted vertex
		if (memory >= verN0 || v->index >= verN0 || last->index >= verN0)
		{
			printf("out of range\n");
		}
		ver[memory] = last; 
		memV[last->index] = memory;
		memV[v->index] = -1;
	}
}

void Mesh::delPolygon(std::shared_ptr<Polygon> t) {
	int memory = memT[t->index];
	if (memory >= 0)
	{
		std::shared_ptr<Polygon> last = tri[--triN];
		tri[memory] = last; // the face in the last position now is transfered to the deleted face position, decrement verN
		memT[last->index] = memory;
		memT[t->index] = -1;
	}
}

void Mesh::verticesTransfer(std::shared_ptr<Mesh> new_mesh, std::vector<std::shared_ptr<Vertex>> vertices) {
	// transfer selected vertices from this mesh to to m1
	if (vertices.empty()) // if no selected vertex transfer all
	{
		for (int i = 0; i < verN; i++) // add all vertex in m2 to m1
		{
			new_mesh->addVertex(ver[i]);
		}
		for (int i = 0; i < new_mesh->verN; i++) // delete all vertex in m2 that belongs to m1
		{
			delVertex(new_mesh->ver[i]);
		}
	}
	else
	{
		int vectorSize = vertices.size();
		for (int i = 0; i < vectorSize; i++)
		{
			if (new_mesh == nullptr)
			{
				printf("vazioo");
			}
			new_mesh->addVertex(vertices[i]);
			delVertex(vertices[i]);
		}
	}
}

void Mesh::facesTransfer(std::shared_ptr<Mesh> new_mesh, std::vector<std::shared_ptr<Polygon>> faces) {
	// transfer selected faces from this mesh to to m1
	if (faces.empty())
	{
		for (int i = 0; i < triN; i++) // add all faces in m2 to m1
		{
			new_mesh->addPolygon(tri[i]);
		}
		for (int i = 0; i < new_mesh->triN; i++) // delete all faces in m2 that belongs to m1
		{
			delPolygon(new_mesh->tri[i]);
		}
	}
	else
	{
		int vectorSize = faces.size();
		for (int i = 0; i < vectorSize; i++)
		{
			new_mesh->addPolygon(faces[i]);
			delPolygon(faces[i]);
		}
	}
}

void Mesh::reconstruct(std::shared_ptr<Mesh> new_mesh) { // add faces of the original to the new one
	if (new_mesh == nullptr) // reconstruct itself, it will eliminate all faces that doesnt belong to this mesh
	{
		for (int i = 0; i < triN; i++) {
			std::shared_ptr<Polygon> t = tri[i];
			int v0 = t->ver[0]->group;
			int v1 = t->ver[1]->group;
			int v2 = t->ver[2]->group;
			if (v0 != index || v1 != index || v2 != index) 
			{
				delPolygon(t);
				i--; // se uma das faces eh deletada eh necessario decrementar i
			}
		}
		reset();
	}
	else // reconstruct another mesh, transfering all faces that belongs to the other
	{
		for (int i = 0; i < triN; i++)
		{
			std::shared_ptr<Polygon> t = tri[i];
			int v0 = t->ver[0]->group;
			int v1 = t->ver[1]->group;
			int v2 = t->ver[2]->group;
			if (v0 == v1 && v0 == v2 && v0 == new_mesh->index)
			{
				facesTransfer(new_mesh, { t });
				i--; // se uma das faces eh deletada eh necessario decrementar i
			}
			//else if (v0 != v1 || v0 != v2) // corrigir isso daqui depois, o mesh soh pode ser deletado depois de ter certeza que nao sera mais usado
			//{
			//	delPolygon(t);
			//	i--; // se uma das faces eh deletada eh necessario decrementar i
			//}
		}
		new_mesh->reset();
	}
}

int Mesh::countThreshold() {
	edgeN = 0;
	for (int i = 0; i < verN; i++)
	{
		if (ver[i]->edge) {
			edgeN++;
		}
	}
	return edgeN;
}

/************ Modification and Calculation *****************/
void Mesh::computeNormals() {
	// in each triangle 
	for (int i = 0; i < triN; i++) {
		// calculate the outerproduct of two sides of a triangle
		std::vector<std::shared_ptr<Vertex>> v = tri[i]->ver;
		std::vector<float> A = v[0]->pos; //
		std::vector<float> B = v[1]->pos;
		std::vector<float> C = v[2]->pos;
		//produto vetorial entre AB e AC, normal a superfice
		float cx = (B[1] - A[1]) * (C[2] - A[2]) - (B[2] - A[2]) * (C[1] - A[1]);//??
		float cy = (B[2] - A[2]) * (C[0] - A[0]) - (B[0] - A[0]) * (C[2] - A[2]);//??
		float cz = (B[0] - A[0]) * (C[1] - A[1]) - (B[1] - A[1]) * (C[0] - A[0]);//??
		// normalizacao
		float l = sqrt(cx * cx + cy * cy + cz * cz);
		if (l != 0) {
			tri[i]->nor[0] = cx / l;
			tri[i]->nor[1] = cy / l;
			tri[i]->nor[2] = cz / l;
		}
		// add the obtained cross product vector to the normal of the three vertices forming the triangle, cross product is area * normal
		v[0]->nor[0] = v[0]->nor[0] + tri[i]->nor[0];
		v[0]->nor[1] = v[0]->nor[1] + tri[i]->nor[1];
		v[0]->nor[2] = v[0]->nor[2] + tri[i]->nor[2];
		v[1]->nor[0] = v[1]->nor[0] + tri[i]->nor[0];
		v[1]->nor[1] = v[1]->nor[1] + tri[i]->nor[1];
		v[1]->nor[2] = v[1]->nor[2] + tri[i]->nor[2];
		v[2]->nor[0] = v[2]->nor[0] + tri[i]->nor[0];
		v[2]->nor[1] = v[2]->nor[1] + tri[i]->nor[1];
		v[2]->nor[2] = v[2]->nor[2] + tri[i]->nor[2];
	}

	// normalize all vertex normals
	for (int i = 0; i < verN; i++) {
		float mod;
		mod = sqrt(ver[i]->nor[0] * ver[i]->nor[0] + ver[i]->nor[1] * ver[i]->nor[1] + ver[i]->nor[2] * ver[i]->nor[2]);
		ver[i]->nor[0] = ver[i]->nor[0] / mod;
		ver[i]->nor[1] = ver[i]->nor[1] / mod;
		ver[i]->nor[2] = ver[i]->nor[2] / mod;
	}
}

void Mesh::computeLaplacians() { // calculate the laplacian vector by summing the neighbor distances
	for (int i = 0; i < triN; i++) {
		tri[i]->prepVerticesLaplacian();
	}

	for (int i = 0; i < verN; i++) { // the laplacian vector is the mean value
		ver[i]->computeLaplacian();
	}
}

void Mesh::findNeighbors() { // Find neighbor vertices
	for (int i = 0; i < verN; i++)
	{
		// create vectors of neighbors
		ver[i]->nei = std::vector<std::shared_ptr<Vertex>>(ver[i]->lapN);
		ver[i]->neiN = 0;
		ver[i]->tri = std::vector<int>(ver[i]->lapN);
		ver[i]->triN = 0;
		ver[i]->nei2N = 0;
	}
	for (int i = 0; i < triN; i++) // add neighbor and faces to the vertices
	{
		tri[i]->neiN = 0;
		tri[i]->nei2N = 0;
		tri[i]->nei = std::vector<std::shared_ptr<Polygon>>(3);
		tri[i]->nei2 = std::vector<std::shared_ptr<Polygon>>(3);

		std::shared_ptr<Vertex> v0 = tri[i]->ver[0];
		std::shared_ptr<Vertex> v1 = tri[i]->ver[1];
		std::shared_ptr<Vertex> v2 = tri[i]->ver[2];
		if (v0->group != index || v1->group != index || v2->group != index)
		{
			printf("erro de transferencia \n");
		}
		v0->addNeighbor(v1);
		v0->addNeighbor(v2);
		v1->addNeighbor(v0);
		v1->addNeighbor(v2);
		v2->addNeighbor(v0);
		v2->addNeighbor(v1);

		v0->addFace(tri[i]->index);
		v1->addFace(tri[i]->index);
		v2->addFace(tri[i]->index);
	}
	for (int i = 0; i < verN; i++) // add neighbors to the faces
	{
		int temp = ver[i]->triN;
		std::vector<std::shared_ptr<Polygon>> t = std::vector<std::shared_ptr<Polygon>>(temp);
		for (int j = 0; j < temp; j++)
		{
			t[j] = tri[memT[ver[i]->tri[j]]];
		}
		for (int j = 0; j < temp; j++)
		{
			for (int k = 1; k < j; k++) {
				if (j != k)
				{
					t[j]->addNeighbor(t[k]);
					t[k]->addNeighbor(t[j]);
				}
			}
		}
	}
	for (int i = 0; i < verN; i++) // try to find borders
	{
		ver[i]->checkBorder();
	}
	for (int i = 0; i < triN; i++)
	{
		tri[i]->checkBorder();
		tri[i]->old = false;
	}
	if (ver[0]->origNei.size() == 0)
	{
		for (int i = 0; i < verN; i++)
		{
			ver[i]->origNei = ver[i]->nei;
			ver[i]->origNeiN = ver[i]->neiN;
		}
	}
}

void Mesh::invertNormals() {
	if (invert_normal)
	{
		for (int i = 0; i < verN; i++)
		{
			ver[i]->nor[0] = -ver[i]->nor[0];
			ver[i]->nor[1] = -ver[i]->nor[1];
			ver[i]->nor[2] = -ver[i]->nor[2];
		}
		for (int i = 0; i < triN; i++)
		{
			tri[i]->nor[0] = -tri[i]->nor[0];
			tri[i]->nor[1] = -tri[i]->nor[1];
			tri[i]->nor[2] = -tri[i]->nor[2];
		}
	}
}

void Mesh::smoothSurface(int nTimes) {
	float dt = 0.1; // was based on the difusion law, the object is kind of melting
	for (int j = 0; j < nTimes; j++)
	{
		for (int i = 0; i < verN; i++)
		{
			ver[i]->smooth(dt);
		}
		clearVectors();
	}
}

void Mesh::computeCurvature() {
	for (int i = 0; i < verN; i++)
	{
		ver[i]->curvatureComputation(aveE);
	}
}

void Mesh::setThreshold(int nTimes) {
	thres = THRES + 0.01*nTimes;
	if (thres > 1)
	{
		int temp = int(thres * 100) % (int(1 - THRES) * 100);
		thres = THRES + float(temp) / 100;
	}
	for (int i = 0; i < verN; i++)
	{
		ver[i]->checkThreshold(thres);
	}
}

void Mesh::regionGrowing(std::shared_ptr<Vertex> seed) {
	// recursevely look for neighbors and check if they can be part of the group or not
	// if they can be part, add them to the subgroup
	grow_count++;
	if (grow_count < MAX_GROW)
	{
		for (int i = 0; i < seed->neiN; i++) // look for all the neighbors
		{
			if (seed->neiN > seed->nei.size() || seed->neiN < 0 || i >= seed->nei.size())
			{
				printf("boboo");
			}
			std::shared_ptr<Vertex> neighbor = seed->nei[i];
			if (neighbor == nullptr || edit_mesh == nullptr)
			{
				printf("null");
			}
			if (neighbor->group == index && !neighbor->analysed) // if the point was still not analysed
			{
				if (neighbor->edge) // if belongs to one of the edges
				{
					neighbor->analysed = true; // register as already analysed
				}
				else { // in case it is classified as part of the group
					verticesTransfer(edit_mesh, { neighbor });
					neighbor->analysed = true;
					regionGrowing(neighbor);
				}
			}
		}
	}
}

int Mesh::regionGrowing2(std::shared_ptr<Vertex> seed) {
	grow_count++;
	int neiGroup = index;
	if (grow_count < MAX_GROW) {
		for (int i = 0; i < seed->neiN; i++)
		{
			std::shared_ptr<Vertex> neighbor = seed->nei[i];
			if (neighbor->group == index && !neighbor->analysed) // if the point was still not analysed
			{
				if (neighbor->checkCurvDiff(seed, 220 + thres*100)) // if belongs to one of the edges
				{
					neighbor->analysed = true; // register as already analysed
				}
				else { // in case it is classified as part of the group
					verticesTransfer(edit_mesh, { neighbor });
					neighbor->analysed = true;
					int temp = regionGrowing2(neighbor);
					if (temp != index && temp != neiGroup && neiGroup != index)
					{
						neiGroup = -1;
					}
					else if(index == neiGroup)
					{
						neiGroup = temp;
					}
				}
			}
			else if (neighbor->group != index && neiGroup != -1)
			{
				neiGroup = neighbor->group;
			}
		}
		return neiGroup;
	}
}

int Mesh::regionGrowing3(std::shared_ptr<Vertex> seed, bool add) {
	int neiGroup = index;
	grow_count++;
	grow_count2++;
	if (grow_count < verN/2)
	{
		for (int i = 0; i < seed->origNeiN; i++)
		{
			if (seed->origNei.size() <= i)
			{
				printf("zero");
			}
			else if (seed->origNei[i] == nullptr)
			{
				printf("vazio");
			}
			std::shared_ptr<Vertex> neighbor = seed->origNei[i];
			if (neighbor->group == index && !neighbor->analysed) // if the point was still not analysed
			{
				//printf("n ana");
				if (add)
				{
					//printf("transferiu");
					verticesTransfer(edit_mesh, { neighbor });
				}
				neighbor->analysed = true;
				int temp = regionGrowing3(neighbor, add);
				if (temp != index && temp != neiGroup && neiGroup != index) // caso haja multiplos grupos vizinhos
				{
					//printf("aquiii");
					neiGroup = -1;
				}
				else if (index == neiGroup)
				{
					neiGroup = temp;
				}
			}
			else if (neighbor->group != index && neiGroup != -1)
			{
				//printf("achooo");
				neiGroup = neighbor->group;
			}
		}
		return neiGroup;
	}
	else
	{
		//printf("uuuuuu");
		return -1;
	}
}

//void Mesh::regionGrowing4(int nseed) {
//	// recursevely look for neighbors and check if they can be part of the group or not
//	// if they can be part, add them to the subgroup
//	std::shared_ptr<Vertex> seed = ver[memV[nseed]];
//	//printf("ok");
//	if (!seed->edge) // if belongs to one of the edges
//	{
//		//printf("rg");
//		verticesTransfer(edit_mesh, { seed });
//	}
//	for (int i = 0; i < seed->origNeiN; i++) // look for all the neighbors
//	{
//		std::shared_ptr<Vertex> neighbor = seed->origNei[i];
//		if (neighbor->group == index && !neighbor->analysed) // if the point was still not analysed
//		{
//			neighbor->analysed = true; // register as already analysed
//			regionGrowing4(neighbor->index);
//		}
//	}
//}

void Mesh::regionGrowing4(std::shared_ptr<Vertex> seed) { // intead of recursion apply a loop that storages the pointer in a explicit stack
	for (int i = 0; i < verN; i++)
	{
		ver[i]->analysed = false;
	}
	
	std::vector<std::shared_ptr<Vertex>> stack = std::vector<std::shared_ptr<Vertex>>(verN);
	int stack_call = 0;
	stack[stack_call++] = seed; // push the vertex pointer
	while (stack_call != 0 || stack_call > verN) // !stack.empty()
	{
		seed = stack[--stack_call]; // pop the pointer
		if (!seed->edge) // if belongs to one of the edges
		{
			verticesTransfer(edit_mesh, { seed });
		}
		for (int i = 0; i < seed->origNeiN; i++) // look for all the neighbors
		{
			std::shared_ptr<Vertex> neighbor = seed->origNei[i];
			if (neighbor->group == index && !neighbor->analysed) // if the point was still not analysed
			{
				//printf("booo");
				neighbor->analysed = true; // register as already analysed
				stack[stack_call++] = neighbor;
			}
		}
	}
}

int Mesh::segmentation(std::shared_ptr<Mesh> new_mesh, int mode) {
	// segmentation manager 
	// return -1 if there is 0 or more than 1 neighbor group
	// return number of group if is surrounded by one single group
	for (int i = 0; i < verN; i++)
	{
		ver[i]->analysed = false;
	}
	std::shared_ptr<Vertex> r = randomPoint();
	r->analysed = true;
	edit_mesh = new_mesh;
	grow_count = 0;
	grow_count2 = 0;
	if (mode == CURV_THRES)
	{
		verticesTransfer(new_mesh, { r });
		regionGrowing(r);
		return 0;
	}
	else if (mode == CURV_DIFF)
	{
		verticesTransfer(new_mesh, { r });
		return regionGrowing2(r);
	}
	else if (mode == FIND_PARTS)
	{
		if (edit_mesh != nullptr)
		{
			while (edit_mesh->index != regionGrowing3(r, false) && grow_count2 < verN)
			{
				std::shared_ptr<Vertex> r = randomPoint();
				grow_count2++;
				r->analysed = true;
				grow_count = 1;
				//printf("loop");
			}
			if (grow_count2 >= verN)
			{
				printf("not found");
			}
			for (int i = 0; i < verN; i++)
			{
				ver[i]->analysed = false;
			}
			printf("init transfer");
			grow_count = 0;
			r->analysed = true;
			verticesTransfer(new_mesh, { r });
			return regionGrowing3(r, true);
		}
		else
		{
			int temp = regionGrowing3(r, false);
			while (temp == -1 && grow_count2 < verN)
			{
				std::shared_ptr<Vertex> r = randomPoint();
				grow_count2++;
				r->analysed = true;
				grow_count = 0;
				temp = regionGrowing3(r, false);
			}
			printf("temp: %d", temp);
			return temp;
		}
	}
}

//int Mesh::linkPath(std::shared_ptr<Polygon> init, int end, int current, int maxValue, std::shared_ptr<Mesh> new_mesh) {
//	if (current < maxValue)
//	{
//		for (int i = 0; i < init->nei2N; i++)
//		{
//			std::shared_ptr<Polygon> neighbor = init->nei2[i];
//			if (neighbor->index == end)
//			{
//				return current;
//			}
//			else if (new_mesh == nullptr) // vai contar o caminho mais curto no proprio mesh
//			{
//				if (neighbor->group == index)
//				{
//					int value = linkPath(neighbor, end, current + 1, maxValue, nullptr);
//					if (value < maxValue)
//					{
//						maxValue = value;
//					}
//				}
//			}
//			else if (neighbor->group == new_mesh->index) // vai contar o caminho em new mesh e transferir faces
//			{
//				int value = linkPath(neighbor, end, current + 1, maxValue, new_mesh);
//				if (value < maxValue)
//				{
//					printf("transfer");
//					facesTransfer(new_mesh, { neighbor }); // ainda falta transferir os vertices
//					maxValue = value;
//				}
//			}
//		}
//	}
//	return maxValue;
//}

//int Mesh::countPath(std::shared_ptr<Polygon> init, int end, int current, int maxValue) {
//	if (current > maxValue)
//	{
//		for (int i = 0; i < init->nei2N; i++) {
//			std::shared_ptr<Polygon> neighbor = init->nei2[i];
//			if (neighbor->index == end)
//			{
//				return current;
//			}
//			else if (neighbor->group == index)
//			{
//				int value = countPath(neighbor, end, current + 1, maxValue);
//				if (value < maxValue)
//				{
//					maxValue = value;
//				}
//			}
//		}
//	}
//	return maxValue;
//}
//
//int Mesh::fillPath(std::shared_ptr<Mesh> old_mesh, int max_distance) {
//	for (int i = 0; i < triN; i++)
//	{
//		for (int j = 0; j < i; j++) {
//			int distance = countPath(tri[i], tri[j]->index, 0, max_distance);
//			linkPath(tri[i], tri[j]->index, 0, distance);
//		}
//	}
//}

int Mesh::fillHoles(std::shared_ptr<Mesh> new_mesh) {
	// look for holes (threshold points and others unselected points)
	// try to classify them counting the number of neighbors
	//new_mesh->fitBox();
	printf("raay: %f", new_mesh->max[3]);
	int count = 0;
	bool completed = false;
	while (!completed && count < 100) // until all points have a classification
	{
		count++;
		completed = true;
		for (int i = 0; i < verN; i++) // look for edges 
		{
			int g = ver[i]->surroundingGroup();
			float distance = sqrtf(pow(ver[i]->pos[0] - new_mesh->mid[0], 2) + pow(ver[i]->pos[1] - new_mesh->mid[1], 2));
			if (g == new_mesh->index)// && ver[i]->group == index && ver[i]->pos[2] < new_mesh->max[2] && ver[i]->pos[2] > new_mesh->min[2] && distance < new_mesh->max[3])//&& ver[i]->pos[0] < new_mesh->max[0] && ver[i]->pos[1] < new_mesh->max[1] && ver[i]->pos[2] < new_mesh->max[2] //&& ver[i]->pos[0] > new_mesh->min[0] && ver[i]->pos[1] > new_mesh->min[1] && ver[i]->pos[2] > new_mesh->min[2]
			{
				verticesTransfer(new_mesh, { ver[i] });
				i--;
				completed = false;
			}
		}
	}
}

void Mesh::fitBox() {
	//float max[3], min[3];
	max = std::vector<float>(4, 0);
	min = std::vector<float>(3, 0);
	mid = std::vector<float>(3, 0);
	size = std::vector<float>(3, 0);
	for (int i = 0; i < 3; i++) { // find max and min values
		max[i] = ver[0]->pos[i];
		min[i] = ver[0]->pos[i];
		for (int j = 1; j < verN; j++) {
			float v = ver[j]->pos[i];
			if (v > max[i])
				max[i] = v;
			else if (v < min[i])
				min[i] = v;
		}
		size[i] = max[i] - min[i];
		mid[i] = 0.5f * (max[i] + min[i]);
	}
	max[3] = sqrtf(pow(max[0] - mid[0],2)+ pow(max[1] - mid[1],2)); // max ray of the teeth
	if (size[2] < max_height && size[2] > min_height &&
		max[3]  < max_ray ) //size[0] < max_width  && size[0] > min_width && //size[1] < max_depth  && size[1] > min_depth &&
	{
		is_tooth = true;
		is_gum = false;
		bigger_than_limits = false;
		smaller_than_limits = false;
		for (int i = 0; i < verN; i++)
		{
			if (ver[i]->pos[2] < (2 * lowest_value + highest_value) / 3) {
				is_tooth = false;
				is_gum   = true;
				//printf("pos: %f", ver[i]->pos[2]);
			}
		}
	}
	else
	{
		is_tooth = false;
		is_gum = false;
		if (size[2] > max_height ||
			max[3]  > max_ray)
		{
			bigger_than_limits = true;
		}
		else
		{
			smaller_than_limits = true;
		}
	}
}


/************ Differents Displays Functions *****************/
void Mesh::plotPoints() {
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glShadeModel(GL_SMOOTH);
	glDisable(GL_LIGHTING); // light off

	float r = color[0];
	float g = color[1];
	float b = color[2];
	for (int i = 0; i < verN; i++) 
	{
		glPointSize(3.0f);
		glBegin(GL_POINTS);
		glColor3f(r, g, b);
		glVertex3fv((GLfloat*)ver[i]->pos.data());
		glEnd();
	}

	/*float r = 0;
	float g = 1;
	float b = 0;
	fitBox();
	glPointSize(10.0f);
	glBegin(GL_POINTS);
	glColor3f(r, g, b);
	for (int i = 0; i < verN; i++)
	{
		if (ver[i]->pos[2] < (2*lowest_value + highest_value) / 3)
		{
			glVertex3fv((GLfloat*)ver[i]->pos.data());
			printf("oioioioi");
		}
	}
	glEnd();*/
}

void Mesh::plotVector() {
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glShadeModel(GL_SMOOTH);
	glDisable(GL_LIGHTING); // light off

	axisX = { 1, 0, 0 };
	axisY = { 0, 1, 0 };
	axisZ = { 0, 0, 1 };

	float r = 0;
	float g = 1;
	float b = 0;
	std::vector<float> temp = std::vector<float>(3);
	temp = { 0, 0, 0 };
	for (int i = 0; i < 100; i++)
	{
		glPointSize(10.0f);
		glBegin(GL_POINTS);
		glColor3f(r, g, b);
		glVertex3fv(temp.data());
		temp[0] = temp[0] + axisX[0]*0.05;
		temp[1] = temp[1] + axisX[1]*0.05;
		temp[2] = temp[2] + axisX[2]*0.05;
		glEnd();
	}

	r = 0;
	g = 1;
	b = 0;
	temp = { 0, 0, 0 };
	for (int i = 0; i < 100; i++)
	{
		glPointSize(10.0f);
		glBegin(GL_POINTS);
		glColor3f(r, g, b);
		glVertex3fv(temp.data());
		temp[0] = temp[0] + axisY[0] * 0.05;
		temp[1] = temp[1] + axisY[1] * 0.05;
		temp[2] = temp[2] + axisY[2] * 0.05;
		glEnd();
	}

	r = 0;
	g = 0;
	b = 1;
	temp = { 0, 0, 0 };
	for (int i = 0; i < 100; i++)
	{
		glPointSize(10.0f);
		glBegin(GL_POINTS);
		glColor3f(r, g, b);
		glVertex3fv(temp.data());
		temp[0] = temp[0] + axisZ[0] * 0.05;
		temp[1] = temp[1] + axisZ[1] * 0.05;
		temp[2] = temp[2] + axisZ[2] * 0.05;
		glEnd();
	}
}

void Mesh::flatShading() {
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	
	for (int i = 0; i < triN; i++) {
		std::vector<std::shared_ptr<Vertex>> v = tri[i]->ver;
		glBegin(GL_POLYGON);
		glNormal3fv((GLfloat*)tri[i]->nor.data());
		glVertex3fv((GLfloat*)v[0]->pos.data()); // glVertex3fv((GLfloat*)ver[t[0]].data());
		glVertex3fv((GLfloat*)v[1]->pos.data());
		glVertex3fv((GLfloat*)v[2]->pos.data());
		glEnd();
	}
}

void Mesh::solidColor() {
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glShadeModel(GL_SMOOTH);
	glDisable(GL_LIGHTING);

	float r = color[0];
	float g = color[1];
	float b = color[2];
	for (int i = 0; i < triN; i++)
	{
		std::vector<std::shared_ptr<Vertex>> v = tri[i]->ver;
		glBegin(GL_POLYGON);
		glColor3f(r, g, b);
		glNormal3fv((GLfloat*)tri[i]->nor.data());
		glVertex3fv((GLfloat*)v[0]->pos.data()); // glVertex3fv((GLfloat*)ver[t[0]].data());
		glVertex3fv((GLfloat*)v[1]->pos.data());
		glVertex3fv((GLfloat*)v[2]->pos.data());
		glEnd();
	}
}

void Mesh::showCurvature() {
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glShadeModel(GL_SMOOTH);
	glDisable(GL_LIGHTING); // light off

	for (int i = 0; i < triN; i++) {
		glBegin(GL_POLYGON);
		for (int j = 0; j < 3; j++) {
			std::shared_ptr<Vertex> v = tri[i]->ver[j];
			float c = v->curv; // em escala de cinza 0-1
			glColor3f(c, c, c);
			glVertex3fv((GLfloat*)v->pos.data());
		}
		glEnd();
	}
}

void Mesh::showThreshold() {
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glShadeModel(GL_SMOOTH);
	glDisable(GL_LIGHTING); // light off

	for (int i = 0; i < verN; i++) // consertar aqui acrescentar os pontos out
	{
		glPointSize(1.0f);
		glBegin(GL_POINTS);
		if (ver[i]->edge) {
			glColor3f(0, 1, 0);
			glVertex3fv((GLfloat*)ver[i]->pos.data());
		}
		glEnd();
	}
}

std::shared_ptr<Vertex> Mesh::randomPoint() { // return a random vertex 
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glShadeModel(GL_SMOOTH);
	glDisable(GL_LIGHTING);
	glPointSize(5.0f);

	int ranP = 0;
	if (verN < RAND_MAX)
	{
		ranP = (rand() % verN);
		int edge = ver[ranP]->edge;
		int analysed = ver[ranP]->analysed;
		while (edge || analysed)
		{
			ranP = (rand() % verN);
			edge = ver[ranP]->edge;
			analysed = ver[ranP]->analysed;
		}
	}
	else if (verN < RAND_MAX * RAND_MAX + RAND_MAX)
	{
		ranP = (rand() * rand() + rand()) % verN;
		int edge = ver[ranP]->edge;
		int analysed = ver[ranP]->analysed;
		while (edge || analysed)
		{
			ranP = (rand() * rand() + rand()) % verN;
			edge = ver[ranP]->edge;
			analysed = ver[ranP]->analysed;
		}
	}
	else
	{
		printf("error: too big sample \n");
	}
	std::shared_ptr<Vertex> v = ver[ranP];
	glBegin(GL_POINTS);
	glColor3f(1, 0.3, 0.3);
	glVertex3fv((GLfloat*)v->pos.data());
	glEnd();

	return v;
}

void Mesh::showBorder() {
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glShadeModel(GL_SMOOTH);
	glDisable(GL_LIGHTING);

	for (int i = 0; i < triN; i++)
	{
		glBegin(GL_POLYGON);
		if (tri[i]->border)
		{
			glColor3f(0, 0, 1);
		}
		else
		{
			glColor3f(1, 0, 0);
		}
		std::vector<std::shared_ptr<Vertex>> v = tri[i]->ver;
		glNormal3fv((GLfloat*)tri[i]->nor.data());
		glVertex3fv((GLfloat*)v[0]->pos.data());
		glVertex3fv((GLfloat*)v[1]->pos.data());
		glVertex3fv((GLfloat*)v[2]->pos.data());
		glEnd();
	}
}
