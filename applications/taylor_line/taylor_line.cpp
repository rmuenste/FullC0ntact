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

    std::vector< Vector3<Real> > u_;

    std::vector< Vector3<Real> > force_;

    SoftBody4() : A_(3.2), N_(100),  f_(1.0/60.0), a0_(0.5), l0_(1.0*a0_)
    {
      transform_.setOrigin(Vec3(0,0,0));
      geom_.center_ = Vec3(0,0,0);
      velocity_ = Vec3(0,0,0);

      geom_.vertices_.reserve(N_);
      u_.reserve(N_);
      force_.reserve(N_);
    };

    virtual ~SoftBody4(){};

    Real getContourLength()
    {
      Real contourLength = 0.0;
      for(int i(1); i < N_; ++i) 
      {
        contourLength += (geom_.vertices_[i] - geom_.vertices_[i-1]).mag(); 
      }
      return contourLength;
    }

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

      for(int i(0); i < N_; ++i)
        force_[i] = Vec3(0,0,0);

      // Spring forces
      for(int i(0); i < N_-2; ++i)
      {

        Vec3 t_i = geom_.vertices_[i+1] - geom_.vertices_[i]; 
        Vec3 t_i2 = geom_.vertices_[i+2] - geom_.vertices_[i+1]; 
        Real l_i = t_i.mag(); 
        Real l_i2 = t_i2.mag(); 
        force_[i] = -kd_ * (l_i - l0_) * t_i + kd_ * (l_i2 - l0_) * t_i2;
//        if(i==98)
//        {
//          std::cout << "Spring Force " << i << ": " << force_[i];
//        }

      }

      // Treatment of boundary vertices
      Vec3 t_i = geom_.vertices_[N_-1] - geom_.vertices_[N_-2]; 
      Real l_i = t_i.mag(); 
      force_[N_-2] = -kd_ * (l_i - l0_)*t_i;  
      std::cout << "Spring Force " << 98 << ": " << force_[98];

      force_[N_-1] = force_[N_-2];


      Real L = getContourLength();
      Real p = 10 * L;
      Real kappa = p;

      Real pi = CMath<Real>::SYS_PI;

      // 1 second wave propagation time 
      f_ = 1.0/5.0;
      A_ = 1.0;
      std::vector<Real> alphas;
      Real lambda_c = (2.0 * pi)/(l0_ * N_); 
      for(int i(0); i < N_; ++i)
      {
        Real xx = (i+1) * l0_;
        Real c_nt = A_ * std::sin(2.0 * pi * f_ * t + xx * lambda_c);
        Real alpha = l0_ * c_nt;
        alphas.push_back(alpha);
      }

      // Angular bending forces
      for(int j(2); j < N_-2; ++j)
      {

        Mat3 r;

        Vec3 t_jm2 = geom_.vertices_[j-1] - geom_.vertices_[j-2]; 
        Vec3 t_jm1 = geom_.vertices_[j] - geom_.vertices_[j-1]; 
        Vec3 t_j = geom_.vertices_[j+1] - geom_.vertices_[j]; 
        Vec3 t_j1 = geom_.vertices_[j+2] - geom_.vertices_[j+1]; 

        r.MatrixFromAngles( Vec3(0,0,alphas[j-2]));

        Vec3 term1 = r * t_jm2 - t_jm1;

        r.MatrixFromAngles( Vec3(0,0,alphas[j-1]));
        Mat3 rt = r.GetTransposedMatrix();

        Vec3 term2 = t_j - t_jm1 + (rt * t_j) - (r * t_jm1);

        r.MatrixFromAngles( Vec3(0,0,alphas[j]));
        rt = r.GetTransposedMatrix();

        Vec3 term3 = t_j - (rt * t_j1);

        Vec3 force_b = kappa * (term1 + term2 + term3);

        force_[j] += force_b;

      }

      // handle case 0
      Mat3 r;

      Vec3 t_0 = geom_.vertices_[1] - geom_.vertices_[0]; 
      Vec3 t_1 = geom_.vertices_[2] - geom_.vertices_[1]; 
      r.MatrixFromAngles( Vec3(0,0,alphas[0]));
      Mat3 rt = r.GetTransposedMatrix();
      Vec3 term3 = t_1 - (rt * t_1);
      force_[0] += kappa * term3;
      
      // handle case 1
      r.MatrixFromAngles( Vec3(0,0,alphas[0]));
      rt = r.GetTransposedMatrix();

      Vec3 term2 = t_1 - t_0 + (rt * t_1) - (r * t_0);

      r.MatrixFromAngles( Vec3(0,0,alphas[1]));
      rt = r.GetTransposedMatrix();
      term3 = t_1 - (rt * t_1);

      force_[1] += kappa * (term2 + term3);
      // handle case 99
      Vec3 t_97 = geom_.vertices_[98] - geom_.vertices_[97]; 
      Vec3 t_98 = geom_.vertices_[99] - geom_.vertices_[98]; 
      r.MatrixFromAngles( Vec3(0,0,alphas[97]));
      Vec3 term1 = (r * t_97) - t_98;
////      force_[99] += 0.1 * kappa * term1;
//
//      force_[99] = force_[0];
      std::cout << "Force " << 99 << ": " << force_[99];

      for(auto &u : force_)
      {
        u.x = 0;
      }

    }; 

    void integrate()
    {

      std::vector<Vector3<Real>> &u0 = u_; 
      std::vector<Vector3<Real>> &f0 = force_; 

      for(int i(0); i < N_; ++i)
      {
        Vec3 &vel = u0[i];

        Vec3 &force = f0[i];

        //force -= 0.0125 * vel;

        Real m = 1.0;

        Vec3 &pos = geom_.vertices_[i];
      
        vel = vel + dt_ * force * (1.0/m);

        vel *= 0.98;

        pos = pos + dt_ * vel;

        //force = Vec3(0,0,0);

      }
    }; 

    void applyForce(Real dt)
    {

    }; 

    void step(Real t, Real dt, int it)
    {
      dt_ = dt;

      internalForce(t); 
      
      integrate();
    }

    void init()
    {

      kd_ = 250;

      Real pi = CMath<Real>::SYS_PI;

      Real q = 3.0 * pi/(l0_ * N_); 

      Real xx = 0 * l0_;


      //f_ = 1.0/1.0;
      //A_ = 1.0;
      Real lambda_c = (2.0 * pi)/(l0_ * N_); 
      //Real yy = A_ * std::sin(xx * lambda_c);
      
      Real yy = A_ * std::sin(lambda_c * xx + 0.5 * pi);
      
//      for(int i(0); i < N_; ++i)
//      {
//        Real xx = (i+1) * l0_;
//      }



      geom_.vertices_.push_back(Vector3<Real>(xx, yy, 0));
      u_.push_back(Vec3(0,0,0));
      force_.push_back(Vec3(0,0,0));

      for(int k=1; k < N_; ++k)
      {

        Real x = Real(k) * l0_;

        //Real y = 0; //A_ * std::sin(2.0 * pi * f_ + q * x);
        Real y = A_ * std::sin(x * lambda_c + 0.5 * pi);

        geom_.vertices_.push_back(Vector3<Real>(x,y,0));

        geom_.faces_.push_back(std::pair<int,int>(k-1,k));

        geom_.segments_.push_back(Segment3<Real>(geom_.vertices_[k-1], 
                                                 geom_.vertices_[k]
                                                ));

        u_.push_back(Vec3(0,0,0));
        force_.push_back(Vec3(0,0,0));

      }

      for(auto &v: geom_.vertices_)
      {
        geom_.verticesOld_.push_back(v);
      }

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
      steps_ = 500000;

      dt_ = 0.001;
      time_ = 0.0;
      step_ = 0;
      //grid_.calcVol();
      //std::cout << "> Grid volume: " << grid_.vol_ << std::endl;

      for(int istep(0); istep <= steps_; ++istep)
      {
        std::cout << "Time: " << time_ << "|-----------------------|" << dt_ << "|it: " << istep<< std::endl;
        flagella_.step(time_,dt_, istep);
        CVtkWriter writer;
        std::ostringstream name;
        name << "output/line." << std::setfill('0') << std::setw(5) << istep << ".vtk";
        //writer.WriteParamLine(flagella_.geom_, name.str().c_str());
        if(istep%1000==0)
        writer.WriteParamLine(flagella_.geom_, flagella_.force_, name.str().c_str());
        time_ += dt_;
        step_++;
        std::cout << "Contour length of the swimmer: " << flagella_.getContourLength() << std::endl;
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
