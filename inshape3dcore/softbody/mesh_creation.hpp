#ifndef MESH_CREATION_HPP
#define MESH_CREATION_HPP


#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <vector>

#include <general_definitions.hpp>

MyMesh generatePlaneMesh();
MyMesh generateSimpleMesh(); 

std::vector<DistanceConstraintd> generatePlaneDistanceConstraints(MyMesh& mesh, int solverIterations);
std::vector<BendingConstraintd> generatePlaneBendingConstraints(MyMesh& mesh, int solverIterations);

std::vector<BendingConstraintd> generateBendingConstraints(MyMesh& mesh, int solverIterations);
std::vector<DistanceConstraintd> generateDistanceConstraints(MyMesh& mesh, int solverIterations);

#endif