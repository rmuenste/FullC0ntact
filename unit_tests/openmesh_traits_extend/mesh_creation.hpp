#ifndef MESH_CREATION_HPP
#define MESH_CREATION_HPP

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<> SimpleMesh;

SimpleMesh generatePlaneMesh();

#endif