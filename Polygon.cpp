#include "Polygon.h"



Polygon::Polygon(int i) {
	origVer = std::vector<std::shared_ptr<Vertex>>(3);
	ver = std::vector<std::shared_ptr<Vertex>>(3);
	nor = std::vector<float>(3, 0);
	index = i;
	group = 0;
	old = false;
}

void Polygon::prepVerticesLaplacian() { // sum up the distances in this face into the lap vector
	// create weak pointer
	std::shared_ptr<Vertex> A = ver[0]; //
	std::shared_ptr<Vertex> B = ver[1];
	std::shared_ptr<Vertex> C = ver[2];
	// sum up
	A->lap[0] += (B->pos[0] - A->pos[0]);
	A->lap[1] += (B->pos[1] - A->pos[1]);
	A->lap[2] += (B->pos[2] - A->pos[2]);
	B->lap[0] += (C->pos[0] - B->pos[0]);
	B->lap[1] += (C->pos[1] - B->pos[1]);
	B->lap[2] += (C->pos[2] - B->pos[2]);
	C->lap[0] += (A->pos[0] - C->pos[0]);
	C->lap[1] += (A->pos[1] - C->pos[1]);
	C->lap[2] += (A->pos[2] - C->pos[2]);
	// number of adjacent neigbors, linked by a triangle side
	A->lapN++;
	B->lapN++;
	C->lapN++;
}

void Polygon::back2original() {
	ver[0] = origVer[0];
	ver[1] = origVer[1];
	ver[2] = origVer[2];
}

void Polygon::clearVector() {
	nor = { 0, 0, 0 };
}

void Polygon::addNeighbor(std::shared_ptr<Polygon> n) {
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
			//neiAdded++;
			if (nei[neiN - 1] == nullptr)
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
		if (nei2N <= 3)
		{
			if (!old)
			{
				nei2[nei2N] = n;
				nei2N++; // caso va ser adicionado duas vezes significa que compartilha 2 arestas
			}
		}
	}
}

void Polygon::checkBorder() {
	for (int i = 0; i < 3; i++)
	{
		if (ver[i]->border) {
			border = true;
		}
	}
}