#ifndef GLOBALS_HPP
#define GLOBALS_HPP

float kStretch = 0.5f; 
float kDamp = 0.00125f;
int numX = 4, numY= 4; //these ar the number of quads
const size_t total_points = (numX+1)*(numY+1);
int mysize = 4;
float hsize = mysize/2.0f;
double W = 1.0 / total_points;
int gSolverIterations = 2;

#endif
