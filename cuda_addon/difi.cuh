#ifndef __difi__cuh__
#define __difi__cuh__
#include <3dmodel.h>
#include <distancemap.h>
#include <rigidbody.h>

void cudaCheckError(const char *message, const char *file, const int line);


void copy_mesh(i3d::Model3D *model);

void copy_distancemap(i3d::DistanceMap<double,i3d::cpu> *map);
void transfer_distancemap(i3d::RigidBody *body, i3d::DistanceMap<double,i3d::cpu> *map);
void dmap_test(i3d::RigidBody *body);

#endif
