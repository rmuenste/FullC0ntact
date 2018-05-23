#include "softbody.hpp"


/**
* This function handles the integration of the
* equations of motion by an explicit euler scheme
*/
void SoftBody4::integrate()
{

  std::vector<Vector3> &u0 = u_;
  std::vector<Vector3> &f0 = force_;
  std::vector<Vector3> &fe = externalForce_;

  for (int i(1); i < N_; ++i)
  {
    Vector3 &vel = u0[i];

    Vector3 &force = f0[i];

    Vector3 &extForce = fe[i];

    Vector3 totalForce = force + extForce;

    std::cout <<
      " > Spring force[" << i << "]: " << force;
//    std::cout <<
//      " > Fluid force[" << i << "]: " << extForce;
//    std::cout <<
//      " > Total force[" << i << "]: " << extForce;

    double m = 0.5;
    if (i == 0)
      m = 10000.0;


    Vector3 &pos = geom_[i];

    Vector3 g(0, 3.0 * -0.0981, 0);

    vel.x = vel.x + dt_ * totalForce.x * (1.0 / m);
    vel.y = vel.y + dt_ * totalForce.y * (1.0 / m) + dt_ * g.y;

//    std::cout <<
//      " > Position[" << i << "]: " << pos.y << " + " << dt_ << " * " << g.y << "=" << std::endl;

    pos.x = pos.x + dt_ * vel.x;
    pos.y = pos.y + dt_ * vel.y;

//    std::cout <<
//      " > Position[" << i << "]: " << pos;
//    std::cout <<
//      " > Velocity[" << i << "]: " << vel;

    force = Vector3(0, 0, 0);
    extForce = Vector3(0, 0, 0);
  }
}

/**
* The step function handles the computation of a single
* time step of the soft body physics. First a wind force
* is added then the spring force is computed. Finally,
* we integrate the equations of motion by computing the total
* force and then updating the velocity and position
*/
void SoftBody4::step(double t, double dt, int it)
{
  dt_ = dt;

  windForce(t);

  springForce();

  integrate();
}

void SoftBody4::externalForce(double t)
{

  //    for (int j(1); j < externalForce_.size(); ++j)
  //    {
  //      Vec3 force(0, 0, 0);
  //      externalForce_[j] += force;
  //    }
}

/**
* The function artificially generates a wind-like force
* based on a sinus wave.
*/
void SoftBody4::windForce(double t)
{
  const double pi = 3.14159265358979323846;
  for (int j(1); j < externalForce_.size(); ++j)
  { 
    Vector3 force(0, 0, 0);
    Vector3 pos = geom_[j];
    //force.x = 0.04 * std::abs(std::sin(pos.x + t * 2.0) + std::cos(pos.y + t * 2.0) / 3.0);
    force.x = 1.5 * std::abs(std::sin(2.0 * pi * 0.2 * t + pos.x));
    externalForce_[j] += force;
  }
}


/**
* The spring force function evaluates the
* forces of each spring by calculating the force
* accoring to Hooke's law.
*/
void SoftBody4::springForce()
{
  for (unsigned i(0); i < springs_.size(); ++i)
  {
    SimpleSpringConstraint<Ogre::Real> &spring = springs_[i];

    Vector3 f = spring.evalForce();

    if (spring.i0_ != 0)
      force_[spring.i0_] += f;

    if (spring.i1_ != 0)
      force_[spring.i1_] -= f;
  }
}

