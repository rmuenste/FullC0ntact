#ifndef GENERAL_DEFINITIONS_HPP
#define GENERAL_DEFINITIONS_HPP

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/BaseKernel.hh>

extern double W;
extern int gSolverIterations;

struct BendingConstraint { 

  double restAngle_, k, kPrime;
  int fidx0, fidx1;
  int p1, p2, p3, p4;

  BendingConstraint() : restAngle_(0), k(0), kPrime(0), fidx0(-1), fidx1(-1),
                        p1(0), p2(0), p3(0), p4(0)
  {};

  BendingConstraint(double _restAngle, double _k, 
                    unsigned _f0, unsigned _f1, int vidx[4], int iterations) : 
                    restAngle_(_restAngle), k(_k), 
                    fidx0(_f0), fidx1(_f1), p1(vidx[0]), p2(vidx[1]), p3(vidx[2]), p4(vidx[3]) {
    kPrime = 1.0 - std::pow((1.0 - k), 1.0 / double(iterations) );
  };

};

struct DistanceConstraint { 

  typedef OpenMesh::VectorT<double, 3> VectorType;
  typedef double ScalarType;
  
  double restLength, k, kPrime;
  int edgeIndex;
  int vertexIdx_[2];

  // Man braucht die VertexHandles nicht als EdgeTraits, weil die durch die Kante gegeben sind.
  DistanceConstraint() : restLength(0), k(0), kPrime(0), edgeIndex(-1), vertexIdx_{ -1, -1 } {};

  DistanceConstraint(const DistanceConstraint &copy) : restLength(copy.restLength), k(copy.k), kPrime(copy.kPrime), edgeIndex(copy.edgeIndex) {
    std::copy(copy.vertexIdx_, copy.vertexIdx_ + 2, vertexIdx_);
  };

  DistanceConstraint(double _restLength, double _k, int  eidx, int v0, int v1, int iterations) : restLength(_restLength), k(_k), edgeIndex(eidx), vertexIdx_{ v0, v1 } {
    kPrime = 1.0 - std::pow((1.0 - k), 1.0 / double(iterations) );
  };

  DistanceConstraint& operator=(const DistanceConstraint &copy) {
    restLength = copy.restLength;  k = copy.k; kPrime = copy.kPrime; edgeIndex = copy.edgeIndex;
    std::copy(copy.vertexIdx_, copy.vertexIdx_ + 2, vertexIdx_);
    return *this;
  };


  VectorType computeCorrection(const VectorType& v0, const VectorType& v1, ScalarType w0, ScalarType w1) {

    VectorType dir = v0 - v1;

    double length = dir.norm();

    dir.normalize();

    double inverseMass = w0 + w1;

    double sumInvMass = inverseMass;

    if (inverseMass == 0.0)
      return VectorType(0, 0, 0);

    VectorType dP = 1. / (sumInvMass) * (length - restLength) * dir * kPrime;
#ifdef DEBUGOUTPUT 
    std::cout << "rest_length: " << "<" << restLength << "> len: " << length  << "> norm: " << dir[0] << " " << dir[1] << " " << dir[2] << " 1/invMass " << 1. / (sumInvMass) << " > k_prime: " << kPrime << std::endl;
#endif

    return dP;
  }

};

struct MyTraits : public OpenMesh::DefaultTraits
{
  // store barycenter of neighbors in this member
  typedef OpenMesh::Vec3d Point;
  typedef OpenMesh::Vec3d Normal;
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
