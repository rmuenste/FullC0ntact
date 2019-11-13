#ifndef MESH_CREATION_HPP
#define MESH_CREATION_HPP


#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <vector>

#include "general_definitions.hpp"

MyMesh generatePlaneMesh();
MyMesh generateSimpleMesh(); 

std::vector<DistanceConstraint> generatePlaneDistanceConstraints(MyMesh& mesh, int solverIterations);
std::vector<BendingConstraint> generatePlaneBendingConstraints(MyMesh& mesh, int solverIterations);

std::vector<BendingConstraint> generateBendingConstraints(MyMesh& mesh, int solverIterations);
std::vector<DistanceConstraint> generateDistanceConstraints(MyMesh& mesh, int solverIterations);

#endif