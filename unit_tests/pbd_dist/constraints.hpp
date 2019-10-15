#ifndef CONSTRAINTS_HPP
#define CONSTRAINTS_HPP

extern int gSolverIterations;

//struct BendingConstraint {	VHandle p1, p2, p3, p4;	Real restLength1, restLength2, k, w1, w2, kPrime;
//
//  typedef MyMesh* MyMeshP;
//
//  MyMeshP mesh_;
//
//  BendingConstraint(MyMeshP _mesh, VHandle pa, VHandle pb, VHandle pc, VHandle pd, Real rl1, Real rl2, Real _k) : mesh_(_mesh), p1(pa), p2(pb), p3(pc), p4(pd), restLength1(rl1), restLength2(rl2), k(_k)  {
//
//    w1 = 1.0;
//    w2 = 1.0;
//
//    kPrime = 1.0 - std::pow((1.0 - k), 1.0 / gSolverIterations);
//  };
//
//};

//struct DistanceConstraint {	OpenMesh::VertexHandle p1, p2;	//Real restLength, k, kPrime;
//
//  //MyMesh *mesh_;
//
//  DistanceConstraint(OpenMesh::VertexHandle pa, OpenMesh::VertexHandle pb, double rl, double _k) : p1(pa), p2(pb) {//, restLength(rl), k(_k)  {
//
//    //kPrime = 1.0 - std::pow((1.0 - k), 1.0 / solverIterations);
//  };

//  void resolveConstraint(unsigned i) {
//
//    typedef MyMesh::Point Point;
//    Point pa = mesh_->point(p1);
//    Point pb = mesh_->point(p2);
//
//    Real m = 0.008;
//    Real w = 1.0 / m;
//
//    // This is actually the directional derivative of the distance constraint
//    OpenMesh::Vec3f dir = pa - pb;
//
//    Real length = dir.norm();
//    Real w1 = w, w2 = w;
//    
//    Real invMass = w1 + w2;
//    
//    // The variable dp is the correction of the distance constraint
//    // The last factor <XXX * distanceConstraints_[i].k_prime> is the multiplication with the 
//    // stiffness of the constraint
//    OpenMesh::Vec3f dp = (1.0 / invMass) * (length - restLength) * (dir / length) * kPrime;
//
//    std::cout << "Distance Constraint correction: " << dp << std::endl;
//    //
//    //    // Update the positions with the distance correction
//    //    if (p1 != 0)
//    //      positionEstimate_[p1] -= dp * w1;
//    //
//    //    if (p2 != 0)
//    //      positionEstimate_[p2] += dp * w2;
//    //
//    //  }
//  }
//
//};

#endif
