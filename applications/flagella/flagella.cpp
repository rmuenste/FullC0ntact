#include <iostream>
#include <application.h>
#include <reader.h>
#include <paramline.h>
#include <softbody.hpp>
#include <mymath.h>

#include <distancepointseg.h>

namespace i3d {

  template<>
  class SoftBody4<Real, ParamLine<Real> > : public BasicSoftBody<Real, ParamLine<Real>>
  {
    public:


    int istep_;

    Real N_;

    Real m_;

    Real kd_;

    Real ks_;

    Real a0_;

    Real l0_;

    Real dt_;

    Real rho_;

    Real massAll_;

    Real volume_;

    Vec3 com_;

    Vec3 velocity_;

    Transformationr transform_;

    std::vector< SimpleSpringConstraint<Real> > springs_;

    std::vector< Vector3<Real> > u_;

    std::vector< Vector3<Real> > force_;

    std::vector< Vector3<Real> > externalForce_;

    Real particleSize_;

    SoftBody4() : N_(9), a0_(0.04), l0_(1.0*a0_), u_(N_), force_(N_), externalForce_(N_) 
    {

      transform_.setOrigin(Vec3(0, 0, 0));

      geom_.center_ = Vec3(0, 0, 0);

      geom_.vertices_.reserve(N_);

      velocity_ = Vec3(0, 0, 0);

      particleSize_ = 0.01;

    };

    virtual ~SoftBody4(){};

    void calcCOM()
    {
      com_ = Vec3(0,0,0);
      for (int i(0); i < geom_.vertices_.size(); ++i)
      {
        com_ += geom_.vertices_[i]; 
      }
      com_ *= (1.0/geom_.vertices_.size());
    }

    Vec3 getCOM()
    {
      calcCOM();
      Vec3 transCom = com_ + transform_.getOrigin();
      return transCom; 
    }

    void setOrigin(const Vec3 &origin)
    {
      transform_.setOrigin(origin);
    }

    void calcVolume() 
    {
      rho_ = 1.1;  
      Real vc = 4.0/3.0 * CMath<Real>::SYS_PI * std::pow(0.5,3.0);
      volume_ = 0.5 * geom_.vertices_.size() * vc;
      Real vc2 = 4.0/3.0 * CMath<Real>::SYS_PI * std::pow(5.0,3.0);
      volume_ += vc2;
      massAll_ = volume_ * rho_;
    }

    bool isInBody(const Vec3 &vQuery, int &id) const
    {
      // transform point to softbody coordinate system 
      Vec3 q = vQuery - transform_.getOrigin();

      for (int i(geom_.vertices_.size() - 1); i >= 0; --i)
      {
        if ((geom_.vertices_[i] - q).mag() < particleSize_)
        {
          id = i + 1;
          return true;
        }
      }

      for (int i(0); i <= geom_.vertices_.size() - 2; ++i)
      {
        Segment3<Real> s(geom_.vertices_[i], geom_.vertices_[i + 1]);
        CDistancePointSeg<Real> distPointSeg(vQuery, s);
        Real dist = distPointSeg.ComputeDistance();
        if (dist < 0.005)
        {
          id = geom_.vertices_.size() + i;
          return true;
        }

      }

      return false;
    }

    /**
     * This function computes the velocity 
     * in a query point. It is neccessary that
     * the function is only called for points
     * that have been verified to be inside
     * the soft body.
     */
    Vec3 getVelocity(const Vec3 &vQuery)
    {
      int imin = 0;
      Real dmin = (geom_.vertices_[0] - vQuery).mag();
      for (int i(1); i < geom_.vertices_.size(); ++i)
      {
        Real mmin = (geom_.vertices_[i] - vQuery).mag();
        if (mmin < dmin)
        {
          dmin = mmin;
          imin = i;
        }
      }
      return u_[imin];
    }

    /**
     * This function computes the velocity 
     * in a query point. It is neccessary that
     * the function is only called for points
     * that have been verified to be inside
     * the soft body and that the point is located
     * on a segment between two mass points.
     */
    Vec3 getVelocitySegment(const Vec3 &vQuery, int ind)
    {

      int idx = ind - geom_.vertices_.size();
      Segment3<Real> s(geom_.vertices_[idx], geom_.vertices_[idx + 1]);
      CDistancePointSeg<Real> distPointSeg(vQuery, s);
      Real dist = distPointSeg.ComputeDistance();
      Real param = distPointSeg.m_ParamSegment;

      Segment3<Real> useg(u_[idx], u_[idx+1]);
      return (useg.center_ + param * useg.dir_);

    }

    void storeVertices()
    {
      for (int i(0); i < geom_.vertices_.size(); ++i)
      {
        geom_.verticesOld_[i] = geom_.vertices_[i];
      }
    }

    void configureStroke(Real t, int istep)
    {

    }

    void step(Real t, Real dt, int it)
    {
      dt_ = dt;

      //configureStroke(t,it);

      //externalForce(t);

      windForce(t);

      //internalForce(t,it); 

      springForce();

      integrate();
    }

    void externalForce(Real t)
    {

      for (int j(1); j < externalForce_.size(); ++j)
      {
        Vec3 force(0, 0, 0);
        externalForce_[j] += force;
      }
    }

    void windForce(Real t)
    {

      for (int j(1); j < externalForce_.size(); ++j)
      {
        Vec3 force(0, 0, 0);
        Vec3 pos = geom_.vertices_[j];
        force.x = 0.02 * std::abs(std::sin(pos.x + t * 5.0) + std::cos(pos.y + t * 5.0) / 3.0);
        externalForce_[j] += force;
      }
    }

    void springForce()
    {
      for (unsigned i(0); i < springs_.size(); ++i)
      {
        SimpleSpringConstraint<Real> &spring = springs_[i];

        Vector3<Real> f = spring.evalForce();

        if(spring.i0_ != 0)
          force_[spring.i0_] += f;

        if(spring.i1_ != 0)
          force_[spring.i1_] -= f;
      }
    }

    void internalForce(Real t, int istep)
    {

      Real dt = dt_;

      for (int k = 1; k < N_; ++k)
      {
        Vec3 &force = force_[k];
        Vec3 pos = geom_.vertices_[k];
        force.x += 0.1 * std::abs(std::sin(pos.x + t * 5.0) + std::cos(pos.y + t * 5.0) / 3.0);
      }

    };

    void applyForce(Real dt)
    {

    };

    void init()
    {

      Real xx = 0.796354;
      Real yy = 0.4;

      ks_ = 10.0;
      kd_ = -0.2;

      geom_.vertices_.push_back(
        Vector3<Real>(xx,
          yy,
          0));

      for (int k = 1; k < N_; ++k)
      {

        Real y = 0.4 - (Real(k) * l0_);

        Real x = xx;

        geom_.vertices_.push_back(Vector3<Real>(x, y, 0));

        geom_.faces_.push_back(std::pair<int, int>(k - 1, k));

        geom_.segments_.push_back(
          Segment3<Real>(geom_.vertices_[k - 1],
            geom_.vertices_[k]
            ));

        springs_.push_back(
          SimpleSpringConstraint<Real>(ks_, kd_, l0_, k - 1, k,
            &geom_.vertices_[k - 1],
            &geom_.vertices_[k],
            &u_[k - 1],
            &u_[k]
            ));

      }

      Real kb = 16.0;
      for (int k(0); k < N_ - 2; k++)
      {
        springs_.push_back(
          SimpleSpringConstraint<Real>(kb, kd_, 2.0*l0_, k, k + 2,
            &geom_.vertices_[k],
            &geom_.vertices_[k + 2],
            &u_[k],
            &u_[k + 2]
            ));
      }

      for (auto &v : geom_.vertices_)
      {
        geom_.verticesOld_.push_back(v);
      }

    };

    void integrate()
    {

      std::vector<Vector3<Real>> &u0 = u_;
      std::vector<Vector3<Real>> &f0 = force_;
      std::vector<Vector3<Real>> &fe = externalForce_;

      for (int i(1); i < N_; ++i)
      {
        Vec3 &vel = u0[i];

        Vec3 &force = f0[i];

        Vec3 &extForce = fe[i];

        Vec3 totalForce = force + extForce;

          std::cout << 
            " > Spring force[" << i << "]: " << force;
          std::cout << 
            " > Fluid force[" << i << "]: " << extForce;
          std::cout << 
            " > Total force[" << i << "]: " << extForce;

        Real m = 0.01;
        if (i == 0)
          m = 10000.0;


        Vec3 &pos = geom_.vertices_[i];

        Vec3 g(0, -0.981, 0);

        vel.x = vel.x + dt_ * totalForce.x * (1.0 / m);
        vel.y = vel.y + dt_ * totalForce.y * (1.0 / m) + dt_ * g.y;

          std::cout <<
            " > Position[" << i << "]: " << pos.y << " + " << dt_ << " * " << g.y << "=" << std::endl;

        pos.x = pos.x + dt_ * vel.x;
        pos.y = pos.y + dt_ * vel.y;

          std::cout <<
            " > Position[" << i << "]: " << pos;
          std::cout <<
            " > Velocity[" << i << "]: " << vel;

        force = Vec3(0, 0, 0);
        extForce = Vec3(0, 0, 0);
      }
    }; 
  };

  class Flagella : public Application<> {

  public:

    SoftBody4<Real, ParamLine<Real>> flagella_;

    int steps_;

    Real dt_;
    Real time_;
    int step_;

    Flagella() : Application() {

    }

    void init(std::string fileName) {

      using namespace std;

      FileParserXML myReader;

      //Get the name of the mesh file from the
      //configuration data file.
      myReader.parseDataXML(this->dataFileParams_, fileName);

      if (hasMeshFile_)
      {
        std::string fileName;
        grid_.initMeshFromFile(fileName.c_str());
      }
      else
      {
        if (dataFileParams_.hasExtents_)
        {
          grid_.initCube(dataFileParams_.extents_[0], dataFileParams_.extents_[2],
            dataFileParams_.extents_[4], dataFileParams_.extents_[1],
            dataFileParams_.extents_[3], dataFileParams_.extents_[5]);
        }
        else
          grid_.initCube(xmin_, ymin_, zmin_, xmax_, ymax_, zmax_);
      }

      configureRigidBodies();
      
    }

    void run()
    {
      flagella_.init();
      flagella_.istep_ = 0;
      steps_ = 0;

      dt_ = 0.01;
      time_ = 0.0;
      step_ = 0;
      //grid_.calcVol();
      //std::cout << "> Grid volume: " << grid_.vol_ << std::endl;

      for(int istep(0); istep <= steps_; ++istep)
      {
        flagella_.step(time_,dt_, istep);
        std::cout << "===============================================================" << std::endl;
        std::cout << "> Step: " << istep << std::endl;
        std::cout << "===============================================================" << std::endl;
        CVtkWriter writer;
        std::ostringstream name;
        name << "output/line." << std::setfill('0') << std::setw(5) << istep << ".vtk";
        writer.WriteParamLine(flagella_.geom_, name.str().c_str());
        time_ += dt_;
        step_++;
        flagella_.istep_ = step_;
      }
    }
  };
}

using namespace i3d;

int main()
{
  Flagella myApp;
  myApp.init("start/sampleRigidBody.xml");
  myApp.run();
  
  return 0;
}
