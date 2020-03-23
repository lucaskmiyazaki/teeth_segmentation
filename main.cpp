// alterar o constructor do meh pra alterarmos os parametros de tamanho do dente


#include <stdio.h> // biblioteca padrao no c
#include <math.h>
#include <iostream>
#include <GL/glut.H>
#include <cstdlib> 
#include <vector>
#include <memory>
#include <time.h>   


#include "Mesh.h"
#include "Polygon.h"
#include "Eigen/Dense.h"
#include "Eigen/Core"
#include "Eigen/SVD"
using namespace std;
using Eigen::MatrixXd;
#define TOOTH_PTS 1000
#define N_TEETH 10
#define MAX_N_GROUPS 50
#define MAX_NO_RESULTS 5
#define FILE "data/OrthoTrios_upper_new.txt"


// OpenGl Parameters
int mouseX, mouseY;
float qw, qx, qy, qz;
float zoom;
float shiftX, shiftY;
int width, height;

// Booleans
bool show_flat, show_curv, show_thr, reg_grow, select, final;

// Mesh groups 
std::vector<std::shared_ptr<Mesh>> mesh;
int segN, showing_group, previous_group, vectorSize; // number of groups, index of showing group

// Segmentation Evaluation
int no_results;
int bigN, smallN;
float scale = 1;


void fillHoles () {
	
	// find all points that belongs to the base (reg grow n pode ter threshhold, n pode ter pertencer a outro ponto)
	std::shared_ptr<Mesh> temp = std::make_shared<Mesh>(mesh[0], -2);
	mesh[0]->edit_mesh = temp;
	mesh[0]->regionGrowing4(mesh[0]->lowest_point);
	mesh[0]->verticesTransfer(temp, { mesh[0]->lowest_point });
	printf("ver temp: %d", temp->verN);
	printf("ver 0: %d", mesh[0]->verN);
	// add points to each group and then to the base (ou ordem contraria), tem q colocar a condicao de previous grupos pra n ficar alterando os grupos prontos
	bool completed = false;
	while (!completed)
	{
		printf("\n");
		completed = true;
		for (int i = 0; i < mesh[0]->verN; i++) // classify threshold
		{
			int g = mesh[0]->ver[i]->surroundingGroup();
			if (g < 0)
			{
				mesh[0]->verticesTransfer(temp, { mesh[0]->ver[i] });
				i--;
				completed = false;
			}
			else if (g <= segN && g!=0 && g > previous_group)
			{
				mesh[0]->verticesTransfer(mesh[g], { mesh[0]->ver[i] });
				i--;
				completed = false;
			}
		}
	}
	printf("ver temp: %d", temp->verN);
	temp->verticesTransfer(mesh[0], {});

	for (int i = 1 + previous_group; i < segN; i++)
	{
		mesh[0]->reconstruct(mesh[i]);
		printf("ver %d: %d", i, mesh[i]->verN);
	}
	mesh[0]->reconstruct(nullptr);
	printf("ver 0: %d", mesh[0]->verN);
}

float segmentaionQuality() {
	float q;
	for (int i = 1; i < segN+1; i++)
	{
		q += mesh[i]->mid[2];
	}
	q = q / segN;
	return q;
}

void resetVar() {
	// setting booleans
	show_flat = true;
	show_curv = false;
	show_thr = false;
	reg_grow = false;
	select = false;
	final = false;

	// set scale
	if (bigN > smallN)
	{
		scale = scale * 1.1;
		printf("maior scale %f \n", scale);
	}
	else if (smallN > bigN)
	{
		scale = scale * 0.9;
		printf("menor scale %f \n", scale);
	}

	// setting mesh var
	char* file;
	file = FILE;
	vectorSize = 30;
	mesh = std::vector<std::shared_ptr<Mesh>>(vectorSize);
	mesh[0] = std::make_shared<Mesh>(file, MAX_HEIGHT * scale, MAX_WIDTH * scale, MAX_DEPTH * scale, MIN_HEIGHT * scale, MIN_WIDTH * scale, MIN_DEPTH * scale, MAX_RAY * scale);
	segN = 0;
	showing_group = 0;
	previous_group = 0;
	no_results = 0;
	bigN = 0;
	smallN = 0;
}

/********************** Functions Set as Glut Events ***********************/
void myinit() {
	GLfloat mat_specular[] = { 0.5, 0.5, 0.5, 1.0 };
	GLfloat mat_diffuse[] = { 0.5, 0.5, 1.0, 1.0 };
	GLfloat mat_ambient[] = { 0.5, 0.5, 0.5, 1.0 };
	GLfloat mat_shininess = { 20.0 };
	GLfloat light_ambient[] = { 0.1, 0.1, 0.1, 1.0 };
	GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };

	/* set up ambient, diffuse, and specular components for light 0 */
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

	/* define material proerties for front face of all polygons */
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialf(GL_FRONT, GL_SHININESS, mat_shininess);

	glShadeModel(GL_SMOOTH);   /* smooth shading */
	glEnable(GL_LIGHTING); /* enable lighting */
	glEnable(GL_LIGHT0);   /* enable light 0 */
	glEnable(GL_DEPTH_TEST); /* enable z buffer */
	glDepthFunc(GL_LEQUAL);

	glClearColor(1.0, 1.0, 1.0, 1.0);
}

void myReshape(int w, int h) {
	width = w;
	height = h;

	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (w <= h)
		glOrtho(-2.0 / zoom, 2.0 / zoom,
			-2.0 * (GLfloat)h / (GLfloat)w / zoom,
			2.0 * (GLfloat)h / (GLfloat)w / zoom,
			-10.0, 10.0);
	else
		glOrtho(-2.0 * (GLfloat)w / (GLfloat)h / zoom,
			2.0 * (GLfloat)w / (GLfloat)h / zoom,
			-2.0 / zoom, 2.0 / zoom, -10.0, 10.0);
	glMatrixMode(GL_MODELVIEW);
}

void display(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glTranslatef(shiftX, shiftY, 0);
	float s = (float)sqrt(qx * qx + qy * qy + qz * qz);
	if (s != 0)
		glRotatef(2.0f * (float)acos(qw) * 180 / 3.1412f, qx / s, qy / s, qz / s);

	
	if (show_flat)
	{
		mesh[0]->flatShading();
		mesh[0]->plotVector();
		mesh[0]->lowest_point->plot();
		mesh[0]->highest_point->plot();
		//printf("menor: %d, maior %d", mesh[0]->lowest_point->index, mesh[0]->highest_point->index);
		//mesh[0]->plotPoints();
	}
	else if (show_curv)
	{
		mesh[0]->showCurvature();
	}
	else if (show_thr)
	{
		mesh[0]->showCurvature();
		mesh[0]->showThreshold();
	}
	else if (reg_grow)
	{
		mesh[0]->showCurvature();
		for (int i = 1; i < segN + 1; i++)
		{
			mesh[i]->plotPoints();
		}
	}
	else if (select)
	{
		mesh[showing_group]->flatShading(); 
		//mesh[showing_group]->showBorder();
	}
	else if (final) {
		for (int i = 0; i < segN + 1; i++)
		{
			mesh[i]->solidColor();
		}
		//mesh[showing_group]->showBorder();
	}

	glFlush();
	glutSwapBuffers();
}

void mouse(int btn, int state, int x, int y) {
	if (state == GLUT_DOWN) {
		mouseX = x;
		mouseY = y;
	}
	else if (btn == GLUT_LEFT_BUTTON) {
		float mx = -0.0025f * (x - mouseX) / zoom;
		float my = 0.0025f * (y - mouseY) / zoom;

		//‰ñ“]
		float c = (float)cos(my);
		float s = (float)sin(my);

		float rw = c * qw - s * qx;
		float rx = c * qx + s * qw;
		float ry = c * qy - s * qz;
		float rz = c * qz + s * qy;

		c = (float)cos(mx);
		s = (float)sin(mx);

		qw = c * rw - s * ry;
		qx = c * rx + s * rz;
		qy = c * ry + s * rw;
		qz = c * rz - s * rx;

		float n = (float)sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
		if (n != 0) {
			qw /= n;
			qx /= n;
			qy /= n;
			qz /= n;
		}
		else {
			qw = 1.0f;
			qx = qy = qz = 0.0f;
		}
		display();
	}
	else if (btn == GLUT_RIGHT_BUTTON) { //Šg‘åk¬
		zoom -= 0.0025f * (y - mouseY);
		if (zoom > 20.0f) zoom = 20.0f;
		else if (zoom < 0.05f) zoom = 0.05f;
		myReshape(width, height);
		display();
	}
	else if (btn == GLUT_MIDDLE_BUTTON) { //•½sˆÚ“®
		shiftX += 0.0025f * (x - mouseX) / zoom;
		shiftY += 0.0025f * (y - mouseY) / zoom;
		display();
	}
}

void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 't':
		if (show_flat)
		{
			printf("move up side down\n");
			mesh[0]->upsidedown = true;
			for (int i = 0; i < mesh[0]->verN; i++)
			{
				mesh[0]->ver[i]->origPos[1] = -mesh[0]->ver[i]->origPos[1];
				mesh[0]->ver[i]->origPos[2] = -mesh[0]->ver[i]->origPos[2];
			}
			mesh[0]->reset();
			display();
		}
		break;
	case 'n':
		if (show_flat)
		{
			printf("invert normals\n");
			mesh[0]->invert_normal = true;
			mesh[0]->clearVectors();
			display();
		}
		else
		{
			printf("not possible operation\n");
		}
		break;
	case 'p':
		if (show_flat)
		{
			printf("Laplacian Smoothing\n");
			mesh[0]->smoothSurface(rand() % 20);
			printf("show curvature\n");
			show_flat = false;
			show_curv = true;
			mesh[0]->computeCurvature();
		}
		else if (show_curv)
		{
			printf("set threshold\n");
			show_curv = false;
			show_thr = true;
			mesh[0]->setThreshold(rand() % 20);
		}
		else if ((reg_grow || show_thr) && (mesh[0]->verN - mesh[0]->countThreshold()) > TOOTH_PTS && segN < MAX_N_GROUPS)
		{
			printf("region growing %d\n", segN+1);
			show_thr = false;
			reg_grow = true;
			if (vectorSize -1 <= segN)
			{
				printf("resizing \n");
				mesh.push_back(std::make_shared<Mesh>(mesh[0], segN));
				segN++;
				vectorSize++;
			}
			else
			{
				mesh[++segN] = std::make_shared<Mesh>(mesh[0], segN);
			}
			int temp = mesh[0]->segmentation(mesh[segN], CURV_THRES);
			mesh[segN]->fitBox();
			if (mesh[segN]->bigger_than_limits)
			{
				bigN++;
			}
			else if (mesh[segN]->smaller_than_limits)
			{
				smallN++;
			}
		}
		else if (showing_group < segN && reg_grow)
		{
			printf("mesh selection\n");
			mesh[0]->reconstruct(mesh[++showing_group]);
			mesh[showing_group]->fitBox();
			select = true;
			reg_grow = false;
		}
		else if (showing_group < segN && select)
		{
			if (mesh[showing_group]->is_tooth)
			{
				printf("aproved mesh %d\n", showing_group);
				printf("width: %f, depth: %f, height: %f \n", mesh[showing_group]->size[0], mesh[showing_group]->size[1], mesh[showing_group]->size[2]);
				mesh[0]->reconstruct(mesh[++showing_group]);
				mesh[showing_group]->fitBox();
			}
			else
			{
				if (mesh[showing_group]->is_gum)
				{
					printf("part of the gum ");
				}
				if (mesh[showing_group]->bigger_than_limits)
				{
					printf("too big sample ");
				}
				else if (mesh[showing_group]->smaller_than_limits)
				{
					printf("too small sample ");
				}
				printf("reproved mesh %d\n", showing_group);
				mesh[showing_group]->verticesTransfer(mesh[0], {}); // transfer all vertices back to mesh 0
				mesh[showing_group]->facesTransfer(mesh[0], {}); // transfer all vertices back to mesh 0
				mesh[segN--]->verticesTransfer(mesh[showing_group], {});
				mesh[0]->reconstruct(mesh[showing_group]);
				mesh[showing_group]->fitBox();
			}
		}
		else if (showing_group == segN && select)
		{
			if (mesh[showing_group]->is_tooth)
			{
				printf("aproved mesh %d\n", showing_group);
				printf("width: %f, depth: %f, height: %f \n", mesh[showing_group]->size[0], mesh[showing_group]->size[1], mesh[showing_group]->size[2]);
			}
			else
			{
				if (mesh[showing_group]->is_gum)
				{
					printf("part of the gum ");
				}
				if (mesh[showing_group]->bigger_than_limits)
				{
					printf("too big sample ");
				}
				else if (mesh[showing_group]->smaller_than_limits)
				{
					printf("too small sample ");
				}
				printf("reproved mesh %d\n", showing_group);
				mesh[showing_group]->verticesTransfer(mesh[0], {});
				mesh[showing_group]->facesTransfer(mesh[0], {}); // transfer all vertices back to mesh 0
				//mesh[showing_group] = mesh[segN--];
				mesh[segN--]->verticesTransfer(mesh[showing_group], {});
				showing_group--;
			}
			if (segN >= N_TEETH)
			{
				final = true;
				select = false;
				no_results = 0;
				printf("quality of the segmentation: %f \n", segmentaionQuality());
				printf("number of groups: %d \n", segN);
				printf("ver: %d, tri: %d \n", mesh[0]->verN, mesh[0]->triN);
			}
			else if (no_results > MAX_NO_RESULTS) // back to initial config
			{
				printf("no results in attempt %d \n", no_results);
				printf("reset all variables \n");
				if (mesh[0]->invert_normal == true)
				{
					resetVar();
					mesh[0]->invert_normal = true;
					mesh[0]->clearVectors();
				}
				else
				{
					resetVar();
				}
			}
			else if (previous_group == segN)
			{
				no_results++;
				printf("no results in attempt %d \n", no_results);
				select = false;
				show_flat = true;
				previous_group = segN;
			}
			else
			{
				printf("resegment ver: %d, tri: %d \n", mesh[0]->verN, mesh[0]->triN);
				no_results = 0;
				select = false;
				show_flat = true;
				previous_group = segN;
			}
		}
		else
		{
			printf("not possible operation\n");
		}
		display();
		break;
	}
}


/********************** Main Loop ***********************/
int main(int argc, char** argv) {

	// setting parameters
	qw = 1;
	qx = qy = qz = 0;
	zoom = 1;
	shiftX = shiftY = 0;
	srand(time(NULL));
	
	// inicializando a janela e o openGl
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); // display mode
	glutInitWindowSize(800, 800); // tamanho da janela
	glutCreateWindow("Teeth Segmentation"); // title
	myinit(); // info do mesh (light, material, color)
	glutReshapeFunc(myReshape); // set a function to be called when window size is changed
	glutDisplayFunc(display);  // set a function for window update
	glutMouseFunc(mouse); // set a function to be called when mouse is clicked
	glutKeyboardFunc(keyboard);// set a function to be called when keyboard is pressed
				  
	resetVar();

	// Run openGl
	glutMainLoop(); //


	
	return 0;
}

