#pragma once

// STD C++ includes
#include <iostream>
#include <vector>

// Ogre includes
#include <OgreVector3.h>

#include "springconstraint.hpp"

using namespace Ogre;

struct SpringConfiguration {

  SpringConfiguration(int _N, double _kd, double _ks, double _a0, double _l0) : N(_N), kd(_kd), ks(_ks), a0(_a0), l0(_l0)
  {

  }

  int N;

  double kd;

  double ks;

  double a0;

  double l0;

};

class SoftBody4
{
public:

  int istep_;

  double N_;

  double m_;

  double kd_;

  double ks_;

  double a0_;

  double l0_;

  double dt_;

  double rho_;

  double massAll_;

  double volume_;

  Vector3 com_;

  std::vector< SimpleSpringConstraint<Real> > springs_;

  std::vector<Vector3> geom_; 

  std::vector<Vector3> u_;

  std::vector<Vector3> force_;

  std::vector<Vector3> externalForce_;

  double particleSize_;

  SoftBody4() : N_(9), ks_(10.0), kd_(-0.2), a0_(8.0), l0_(1.0*a0_), geom_(N_), u_(N_), force_(N_), externalForce_(N_)
  {

    particleSize_ = 0.01;

  };

  SoftBody4(const SpringConfiguration &sc) : N_(sc.N), ks_(sc.ks), kd_(sc.kd), a0_(sc.a0), l0_(1.0*a0_), geom_(N_), u_(N_), force_(N_), externalForce_(N_)
  {

    particleSize_ = 0.01;

  };

  virtual ~SoftBody4() {};

  void integrate();

  void step(double t, double dt, int it);

  void externalForce(double t);

  void windForce(double t);

  void springForce();

};

struct InitSpringMesh {

  SpringConfiguration &springConf;

  std::vector< SimpleSpringConstraint<Real> > &springs;

  std::vector< Vector3 > &geom;

  std::vector<Vector3> &u;

  std::vector<Vector3> &force;

  std::vector<Vector3> &externalForce;

  InitSpringMesh(SpringConfiguration &_springConf, std::vector< SimpleSpringConstraint<Real> > &_springs, std::vector<Vector3> &_geom,
    std::vector<Vector3> &_u, std::vector<Vector3> &_force, std::vector<Vector3> &_externalForce) : springConf(_springConf), springs(_springs), geom(_geom), u(_u), force(_force), externalForce(_externalForce)
  {

  };

  void init() {

    double xx = 0.0;
    double yy = 120.0;

    //      ks = 10.0;
    //      kd = -0.2;

    geom[0] = (
      Vector3(xx,
        yy,
        0));

    for (int k = 1; k < springConf.N; ++k)
    {

      double y = yy - (double(k) * springConf.l0);

      double x = xx;

      geom[k] = Vector3(x, y, 0);

      std::cout << geom[k].x << " " << geom[k].y << " " << geom[k].z << std::endl;

      springs.push_back(
        SimpleSpringConstraint<Ogre::Real>(springConf.ks, springConf.kd, springConf.l0, k - 1, k,
          &geom[k - 1],
          &geom[k],
          &u[k - 1],
          &u[k]
          ));

    }

    double kb = 160.0;
    for (int k(0); k < springConf.N - 2; k++)
    {
      springs.push_back(
        SimpleSpringConstraint<Ogre::Real>(kb, springConf.kd, 2.0*springConf.l0, k, k + 2,
          &geom[k],
          &geom[k + 2],
          &u[k],
          &u[k + 2]
          ));
    }

  }

};

