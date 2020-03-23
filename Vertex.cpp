#include "Polygon.h"


Vertex::Vertex(int i) {
	index = i;
	nor = std::vector<float>(3, 0);
	pos = std::vector<float>(3, 0);
	origPos = std::vector<float>(3, 0);
	lap = std::vector<float>(3, 0);
	neiN = 0;
	curv = 0;
	edge = false;
	analysed = false;
	group = 0;
}

float Vertex::getPos(int n) {
	return pos[n];
}

void Vertex::computeLaplacian() {
	if (neiN != 0) {
		lap[0] = lap[0] / lapN;
		lap[1] = lap[1] / lapN;
		lap[2] = lap[2] / lapN;
	}
	else {
		lap[0] = 0;
		lap[1] = 0;
		lap[2] = 0;
	}
}

void Vertex::curvatureComputation(float aveE) {
	float dot = lap[0] * nor[0] + lap[1] * nor[1] + lap[2] * nor[2];
	curv = -5.0 * dot / aveE + 0.5; // em escala de cinza 0-1
}

void Vertex::back2original() {
	pos = origPos;
}

void Vertex::clearVector() {
	nor = { 0,0,0 };
	lap = { 0,0,0 };
	lapN = 0;
	curv = 0;
	edge = false;
	analysed = false;
}

void Vertex::checkThreshold(float threshold) {
	if (curv < threshold) {
		edge = true;
	}
	else
	{
		edge = false;
	}
}

bool Vertex::checkCurvDiff(std::shared_ptr<Vertex> n, float threshold) {
	float dist = pow(pos[0] - n->pos[0], 2) + pow(pos[1] - n->pos[1], 2) + pow(pos[2] - n->pos[2], 2);
	float f = fabs(curv - n->curv) / dist;
	//printf("f: %f", f);
	if (f < threshold)
	{
		return false;
	}
	else {
		return true;
	}
}

void Vertex::smooth(float dt) {
	pos[0] = pos[0] + dt * lap[0];
	pos[1] = pos[1] + dt * lap[1];
	pos[2] = pos[2] + dt * lap[2];
}

void Vertex::addNeighbor(std::shared_ptr<Vertex> n) {
	bool same = false;
	for (int j = 0; j < neiN; j++) // check if t[0] was already registered as t[1] neighbor
	{
		if (nei[j] == nullptr)
		{
			printf("error: memory allocation\n");
		}
		else if (nei[j]->index == n->index)
		{
			same = true;
			break;
		}
	}
	if (!same) // if it is not the same add them as each other neighbor
	{
		if (neiN >= nei.size())
		{
			nei.push_back(n);
			neiN++;
			if (nei[neiN-1] == nullptr)
			{
				printf("error: memory allocation\n");
			}
		}
		else
		{
			nei[neiN++] = n;
			if (nei[neiN - 1] == nullptr)
			{
				printf("error: memory allocation\n");
			}
		}
	}
	else
	{
		nei2N++;
	}
}

void Vertex::addFace(int t) {
	if (triN >= tri.size())
	{
		tri.push_back(t);
		triN++;
	}
	else
	{
		tri[triN++] = t;
	}
}

void Vertex::checkBorder() {
	if (nei2N == 3)
	{
		border = false;
	}
	else if (nei2N > 3)
	{
		//printf("%d \n", nei2N);
	}
	else {
		border = true;
	}
}

int Vertex::surroundingGroup() {
	int size = 0;
	int qtd = 0;
	for (int i = 0; i < origNeiN; i++)
	{
		int temp = origNei[i]->group;
		if (temp > size)
		{
			size = temp;
		}
	}
	std::vector<int> g = std::vector<int>(size+1, 0);
	for (int i = 0; i < origNeiN; i++)
	{
		if (origNei[i]->group < 0)
		{
			qtd++;
		}
		else
		{
			g[origNei[i]->group]++;
		}
	}
	int maxValue = qtd;
	int maxIndex = -2;
	for (int i = 0; i < size+1; i++)
	{
		if (g[i] >= maxValue && i != 0)
		{
			maxIndex = i;
			maxValue = g[i];
		}
	}
	if (maxValue == 0)
	{
		return 0;
	}
	if (maxIndex != -2)
	{
		//printf("maxv = %d, %d", maxValue, maxIndex);
	}
	return maxIndex;
}

void Vertex::plot() {
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glShadeModel(GL_SMOOTH);
	glDisable(GL_LIGHTING); // light off

	float r = 1;
	float g = 0;
	float b = 0;
	glPointSize(10.0f);
	glBegin(GL_POINTS);
	glColor3f(r, g, b);
	glVertex3fv((GLfloat*)pos.data());
	glEnd();

	/*std::vector<float> temp = std::vector<float>(3);
	temp = pos;
	for (int j = 0; j < 100; j++)
	{
		temp[0] -= 0.05;
		temp[1] += 2.5;
		for (int i = 0; i < 50; i++)
		{
			temp[1] -= 0.05;
			glPointSize(10.0f);
			glBegin(GL_POINTS);
			glColor3f(r, g, b);
			glVertex3fv((GLfloat*)temp.data());
			glEnd();
		}
	}*/
}