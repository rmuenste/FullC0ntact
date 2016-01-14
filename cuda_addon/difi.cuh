#ifndef __difi__cuh__
#define __difi__cuh__
#include <3dmodel.h>
#include <distancemap.h>
#include <rigidbody.h>
#include <uniformgrid.h>

void cudaCheckError(const char *message, const char *file, const int line);


void copy_mesh(i3d::Model3D *model);

void copy_distancemap(i3d::DistanceMap<double,i3d::cpu> *map);
void transfer_distancemap(i3d::RigidBody *body, i3d::DistanceMap<double,i3d::cpu> *map);
void dmap_test(i3d::RigidBody *body);
void sphere_test(i3d::RigidBody *body, i3d::UniformGrid<i3d::Real,i3d::ElementCell,i3d::VertexTraits<i3d::Real>> &grid);

void transfer_uniformgrid(i3d::UniformGrid<i3d::Real,i3d::ElementCell,i3d::VertexTraits<i3d::Real>> *grid);
void allocate_distancemaps(std::vector<i3d::RigidBody*> &rigidBodies, std::vector<i3d::DistanceMap<i3d::Real>* > &maps);

#endif
