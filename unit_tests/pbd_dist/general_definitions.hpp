#ifndef GENERAL_DEFINITIONS_HPP
#define GENERAL_DEFINITIONS_HPP

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/BaseKernel.hh>

extern double W;
extern int gSolverIterations;

struct DistanceConstraint { double restLength, k, kPrime;
//
  typedef OpenMesh::VectorT<double, 3> VectorType;
  // Man braucht die VertexHandles nicht als EdgeTraits, weil die durch die Kante gegeben sind.
  DistanceConstraint() : restLength(0), k(0), kPrime(0) {};

  DistanceConstraint(double _restLength, double _k) : restLength(_restLength), k(_k) {
    kPrime = 1.0 - std::pow((1.0 - k), 1.0 / 2.0);
  };

  VectorType computeCorrection(const VectorType& v0, const VectorType& v1) {

    VectorType dir = v0 - v1;

    double length = dir.norm();

    dir.normalize();

    double inverseMass = 1.0 / (W);

    double sumInvMass = inverseMass + inverseMass;

    VectorType dP = 1. / (sumInvMass) * (length - restLength) * dir * kPrime;

    return (inverseMass * dP);
  }

};

struct MyTraits : public OpenMesh::DefaultTraits
{
  // store barycenter of neighbors in this member
  EdgeTraits
  {
    public:
      DistanceConstraint dc_;
  //    VertexT() : cog_(Point(0.0f, 0.0f, 0.0f)) { }
 //    const Point& cog() const { return cog_; }
  };
};


typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> MyMesh;
//typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;
typedef MyMesh::VertexHandle VHandle;

#endif
