#ifndef CONSTRAINTS_HPP
#define CONSTRAINTS_HPP

template <typename T>
struct DistanceConstraint { 

  typedef typename T ScalarType;
  typedef OpenMesh::VectorT<ScalarType, 3> VectorType;
  
  T restLength, k, kPrime;
  int edgeIndex;
  int vertexIdx_[2];

  // Man braucht die VertexHandles nicht als EdgeTraits, weil die durch die Kante gegeben sind.
  DistanceConstraint() : restLength(0), k(0), kPrime(0), edgeIndex(-1), vertexIdx_{ -1, -1 } {};

  DistanceConstraint(const DistanceConstraint &copy) : restLength(copy.restLength), k(copy.k), kPrime(copy.kPrime), edgeIndex(copy.edgeIndex) {
    std::copy(copy.vertexIdx_, copy.vertexIdx_ + 2, vertexIdx_);
  };

  DistanceConstraint(ScalarType _restLength, ScalarType _k, int  eidx, int v0, int v1, int iterations) : restLength(_restLength), k(_k), edgeIndex(eidx), vertexIdx_{ v0, v1 } {
    kPrime = 1.0 - std::pow((1.0 - k), 1.0 / ScalarType(iterations) );
  };

  DistanceConstraint& operator=(const DistanceConstraint &copy) {
    restLength = copy.restLength;  k = copy.k; kPrime = copy.kPrime; edgeIndex = copy.edgeIndex;
    std::copy(copy.vertexIdx_, copy.vertexIdx_ + 2, vertexIdx_);
    return *this;
  };


  VectorType computeCorrection(const VectorType& v0, const VectorType& v1, ScalarType w0, ScalarType w1) {

    VectorType dir = v0 - v1;

    ScalarType length = dir.norm();

    dir.normalize();

    ScalarType inverseMass = w0 + w1;

    ScalarType sumInvMass = inverseMass;

    if (inverseMass == 0.0)
      return VectorType(0, 0, 0);

    VectorType dP = 1. / (sumInvMass) * (length - restLength) * dir * kPrime;
#ifdef DEBUGOUTPUT 
    std::cout << "rest_length: " << "<" << restLength << "> len: " << length  << "> norm: " << dir[0] << " " << dir[1] << " " << dir[2] << " 1/invMass " << 1. / (sumInvMass) << " > k_prime: " << kPrime << std::endl;
#endif

    return dP;
  }

};

typedef DistanceConstraint<double> DistanceConstraintd;

template <typename T>
struct BendingConstraint { 

  typedef typename T ScalarType;
  typedef OpenMesh::VectorT<ScalarType, 3> VectorType;

  ScalarType restAngle_, k, kPrime;
  int fidx0, fidx1;
  int p1, p2, p3, p4;

  BendingConstraint() : restAngle_(0), k(0), kPrime(0), fidx0(-1), fidx1(-1),
                        p1(0), p2(0), p3(0), p4(0)
  {};

  BendingConstraint(ScalarType _restAngle, ScalarType _k, 
                    unsigned _f0, unsigned _f1, int vidx[4], int iterations) : 
                    restAngle_(_restAngle), k(_k), 
                    fidx0(_f0), fidx1(_f1), p1(vidx[0]), p2(vidx[1]), p3(vidx[2]), p4(vidx[3]) {
    kPrime = 1.0 - std::pow((1.0 - k), 1.0 / ScalarType(iterations) );
  };

};

typedef BendingConstraint<double> BendingConstraintd;

#endif
