#ifndef GENERAL_DEFINITIONS_HPP
#define GENERAL_DEFINITIONS_HPP

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/BaseKernel.hh>

#include "constraints.hpp"

struct MyTraits : public OpenMesh::DefaultTraits
{
  // store barycenter of neighbors in this member
  typedef OpenMesh::Vec3d Point;
  typedef OpenMesh::Vec3d Normal;
  EdgeTraits
  {
    public:
      DistanceConstraintd dc_;
  //    VertexT() : cog_(Point(0.0f, 0.0f, 0.0f)) { }
 //    const Point& cog() const { return cog_; }
  };
};


typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> MyMesh;
//typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;
typedef MyMesh::VertexHandle VHandle;

#endif
