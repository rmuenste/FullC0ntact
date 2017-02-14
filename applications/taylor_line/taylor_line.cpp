#include <iostream>
#include <application.h>
#include <reader.h>
#include <paramline.h>
#include <softbody.hpp>
#include <mymath.h>

namespace i3d {

  template<>
  class SoftBody4<Real, ParamLine<Real> > : public BasicSoftBody<Real, ParamLine<Real>>
  {
    public:

    Real A_;

    Real q_;

    Real N_;

    Real f_;

    Real m_;

    Real k_tail_;

    Real kd_;

    Real ks_;

    Real a0_;

    Real l0_;

    Real dt_;

    Real rho_;

    Real massAll_;

    Real volume_;

    int n_mid_;

    int n_head_;

    int offset_;

    Vec3 com_;

    Vec3 velocity_;

    Transformationr transform_;

    std::vector< SimpleSpringConstraint<Real> > springs_;

    SoftBody4() : A_(3.2), N_(100),  f_(1.0/60.0), a0_(0.5), l0_(1.0*a0_)
    {
      transform_.setOrigin(Vec3(0,0,0));
      geom_.center_ = Vec3(0,0,0);
      velocity_ = Vec3(0,0,0);
    };

    virtual ~SoftBody4(){};

    void step(Real dt, const Vec3 &force)
    {
      Vec3 pos = transform_.getOrigin();
      Real fx = force.x;
      fx *= 1e-3;

      velocity_.x = velocity_.x + dt * (fx * (1.0/massAll_)); 

      pos.x = pos.x + dt * velocity_.x;

      transform_.setOrigin(pos);
      geom_.center_ = pos;
    }

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

    bool isInBody(const Vec3 &vQuery) const
    {
      // transform point to softbody coordinate system 
      Vec3 q = vQuery - transform_.getOrigin(); 
      for (int i(0); i < geom_.vertices_.size(); ++i)
      {
        if((geom_.vertices_[i] - q).mag() < 0.5)
        {
//          std::cout << "q: " << q << "v: " << geom_.vertices_[i] << "v-q: " << geom_.vertices_[i] - q << std::endl;
          return true;
        }
      }
      return false;
    }

    int isInBodyID(const Vec3 &vQuery) const
    {
      // transform point to softbody coordinate system 
      Vec3 q = vQuery - transform_.getOrigin(); 
      int imin = 0;
      Real dmin = (geom_.vertices_[0] - q).mag(); 
      for (int i(1); i < geom_.vertices_.size(); ++i)
      {
        Real mmin = (geom_.vertices_[i] - q).mag();
        if(mmin < dmin)
        {
          dmin = mmin; 
          imin = i;
        }
      }
      if((geom_.vertices_[imin] - q).mag() < 0.5)
      {
        return imin+1;
      }
      int last = geom_.vertices_.size();
      if((geom_.vertices_[last-1] - q).mag() < 5.0)
        return (last-1);

      return 0;
    }

    Vec3 getVelocity(const Vec3 &vQuery, Real t)
    {
      Vec3 q = vQuery - transform_.getOrigin(); 
      int imin = 0;
      Real dmin = (geom_.vertices_[0] - q).mag(); 
      for (int i(1); i < geom_.vertices_.size(); ++i)
      {
        Real mmin = (geom_.vertices_[i] - q).mag();
        if(mmin < dmin)
        {
          dmin = mmin; 
          imin = i;
        }
      }

      Vec3 velocity = (1.0/dt_) * (geom_.vertices_[imin] - geom_.verticesOld_[imin]);
      Real vx = velocity_.x;
      return Vec3(vx,velocity.y,0);
    }

    void storeVertices()
    {
      for(int i(0); i < geom_.vertices_.size(); ++i)
      {
        geom_.verticesOld_[i] = geom_.vertices_[i];
      }
    }

    void internalForce(Real t)
    {

      const double pi = 3.1415926535897;

      Real q = 4.0 * pi/(l0_ * N_); 


      for(int i(0); i < N_; ++i)
      {

        Real x = Real(i) * a0_;

        Real xl = -2.0 * pi * f_ * t + q * x;

        Real y = A_ * std::sin(xl);

        geom_.vertices_[i]=Vec3(x,y,0);

      }

    }; 

    void applyForce(Real dt)
    {

    }; 

    void step(Real t, Real dt)
    {
      dt_ = dt;
      internalForce(t); 
      integrate();
    }

    void init()
    {


      a0_ = 0.5;

      Real pi = CMath<Real>::SYS_PI;

      Real q = 2.0 * pi/(l0_ * N_); 

      Real xx = 0 * l0_;
      Real yy = A_ * std::sin(-2.0 * pi * f_ + q * xx);


      geom_.vertices_.push_back(Vector3<Real>(xx,
                                              yy,
                                              0));

      for(int k=1; k < N_; ++k)
      {

        Real x = Real(k) * l0_;

        Real y = A_ * std::sin(-2.0 * pi * f_ + q * x);

        geom_.vertices_.push_back(Vector3<Real>(x,y,0));

        geom_.faces_.push_back(std::pair<int,int>(k-1,k));

        geom_.segments_.push_back(Segment3<Real>(geom_.vertices_[k-1], 
                                                 geom_.vertices_[k]
                                                ));

      }

      for(auto &v: geom_.vertices_)
      {
        geom_.verticesOld_.push_back(v);
      }

    }; 

    void integrate()
    {
      
    }; 

  };


  class Flagella : public Application {
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
    }

    void run()
    {
      flagella_.init();
      steps_ = 0;

      dt_ = 0.01;
      time_ = 0.0;
      step_ = 0;
      //grid_.calcVol();
      //std::cout << "> Grid volume: " << grid_.vol_ << std::endl;

      for(int istep(0); istep <= steps_; ++istep)
      {
//        flagella_.step(time_,dt_, istep);
        CVtkWriter writer;
        std::ostringstream name;
        name << "output/line." << std::setfill('0') << std::setw(5) << istep << ".vtk";
        writer.WriteParamLine(flagella_.geom_, name.str().c_str());
//        time_ += dt_;
//        step_++;
//        flagella_.istep_ = step_;
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
