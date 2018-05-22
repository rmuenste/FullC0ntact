#include "softbody.hpp"


  void SoftBody4::step(double t, double dt, int it)
  {
//    dt_ = dt;

//    //configureStroke(t,it);

//    //externalForce(t);

//    windForce(t);

//    //internalForce(t,it); 

//    springForce();

//    integrate();
  }

  void SoftBody4::externalForce(double t)
  {

//    for (int j(1); j < externalForce_.size(); ++j)
//    {
//      Vec3 force(0, 0, 0);
//      externalForce_[j] += force;
//    }
  }

  void SoftBody4::windForce(double t)
  {

//    for (int j(1); j < externalForce_.size(); ++j)
//    {
//      Vec3 force(0, 0, 0);
//      Vec3 pos = geom_.vertices_[j];
//      force.x = 0.02 * std::abs(std::sin(pos.x + t * 5.0) + std::cos(pos.y + t * 5.0) / 3.0);
//      externalForce_[j] += force;
//    }
  }

  void SoftBody4::springForce()
  {
//    for (unsigned i(0); i < springs_.size(); ++i)
//    {
//      SimpleSpringConstraint<double> &spring = springs_[i];

//      Vector3<double> f = spring.evalForce();

//      if (spring.i0_ != 0)
//        force_[spring.i0_] += f;

//      if (spring.i1_ != 0)
//        force_[spring.i1_] -= f;
//    }
  }

  void SoftBody4::internalForce(double t, int istep)
  {

//    double dt = dt_;

//    for (int k = 1; k < N_; ++k)
//    {
//      Vec3 &force = force_[k];
//      Vec3 pos = geom_.vertices_[k];
//      force.x += 0.1 * std::abs(std::sin(pos.x + t * 5.0) + std::cos(pos.y + t * 5.0) / 3.0);
//    }

  };

