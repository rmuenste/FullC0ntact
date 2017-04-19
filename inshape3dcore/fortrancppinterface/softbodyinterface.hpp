#ifndef SOFTBODYINTERFACE_HPP_ZD18N5IP
#define SOFTBODYINTERFACE_HPP_ZD18N5IP

#include <softbody.hpp>

namespace i3d {

  template<>
  class SoftBody4<Real, ParamLine<Real> > : public BasicSoftBody<Real, ParamLine<Real>>
  {
    public:

    int istep_;

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

    int up_;

    int strokeCount_;

    int nForce_;

    Real particleSize_;

    SoftBody4() : A_(3.2), N_(50),  f_(1.0/60.0), a0_(1.0), l0_(1.0*a0_)
    {
      transform_.setOrigin(Vec3(0,0,0));
      geom_.center_ = Vec3(0,0,0);

      geom_.vertices_.reserve(N_);
      u_.reserve(N_);
      force_.reserve(N_);

      velocity_ = Vec3(0,0,0);

      up_ = true;

      strokeCount_ = 0;

      nForce_ = 4;

      particleSize_ = 0.5;
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

      for (int i(geom_.vertices_.size()-1); i >= 0; --i)
      {
        if((geom_.vertices_[i] - q).mag() < particleSize_)
        {
          id=i+1;
          return true;
        }
      }
      return false;
    }

    Vec3 getVelocity(const Vec3 &vQuery,int ind)
    {
      return u_[ind];
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

      Vec3 velocity = u_[imin];

      return velocity;
    }

    void storeVertices()
    {
      for(int i(0); i < geom_.vertices_.size(); ++i)
      {
        geom_.verticesOld_[i] = geom_.vertices_[i];
      }
    }

    void configureStroke(Real t, int istep)
    {
      if(istep == 0)
      {
        up_ = true;
        strokeCount_ = 0;
        return;
      }

      if(strokeCount_ == 1200)
      {
        up_ = !up_;
        //strokeCount_ = 0;
        return;
      }

      strokeCount_++;
    }

    void step(Real t, Real dt, int it)
    {
      dt_ = dt;

      configureStroke(t,it);

      internalForce(t,it); 
      
      integrate();
    }

    void internalForce(Real t, int istep)
    {

      Real A = 4.0;
      Real f = 1.0 / 12.0;
      Real phi = 0 * CMath<Real>::SYS_PI;

      Real sign = 1.0;
      Real scale = 1.0;


      if(strokeCount_ < 1200)
      {
        if (up_)
        {
          sign = 1.0;
          for (int j(0); j < geom_.vertices_.size(); ++j)
          {

            Vec3 fluidForce = myWorld.rigidBodies_[j]->force_;
            Real scale = 1.0;
            if(j >= N_ - nForce_)
            {
              Real fz = A * std::sin(2.0 * CMath<Real>::SYS_PI * f * t + phi);
//              if(myWorld.parInfo_.getId()==1)
//              {
//                std::cout << "> id: [" << j << "] fz: " << fz <<std::endl; 
//              }
//              while(std::abs(fluidForce.z) > fz) 
//              {
//                fluidForce.z *= 0.5;
//              }
              //force_[j].z = sign * fz + fluidForce.z;
              
              u_[j].z = scale * sign * fz;
              if(myWorld.parInfo_.getId()==1)
              {
                std::cout << "> id: [" << j << "] Scale: " << scale <<std::endl; 
                std::cout << "> id: [" << j << "] force: " << force_[j].z <<std::endl; 
                std::cout << "> id: [" << j << "] forceFluid: " << fluidForce.z <<std::endl; 
              }

            }
            else
            {
              force_[j].z = 1e-5 * fluidForce.z;
            }
             
          }
        }
      }


      for(unsigned i(0); i < springs_.size(); ++i)
      {
        SimpleSpringConstraint<Real> &spring = springs_[i];

        Vector3<Real> f = spring.evalForce();

//        if(spring.i0_ >= N_ - nForce_)
//        {
//          if(myWorld.parInfo_.getId()==1)
//          {
//            std::cout << "> ForceDriver " << force_[spring.i0_].z << std::endl; 
//          }
//        }
//
//        if(spring.i1_ >= N_ - nForce_)
//        {
//          if(myWorld.parInfo_.getId()==1)
//          {
//            std::cout << "> ForceDriver " << force_[spring.i1_].z << std::endl; 
//          }
//        }

        force_[spring.i0_] += f;
        force_[spring.i1_] -= f;

      }

    }; 

    void applyForce(Real dt)
    {

    }; 


    void init()
    {

      Real xx = 0 * l0_;
      Real yy = 0;

      ks_ = 320.0;
      kd_ = -0.2;

      geom_.vertices_.push_back(
          Vector3<Real>(xx,
                        yy,
                        0));

      u_.push_back(Vec3(0,0,0));

      force_.push_back(Vec3(0,0,0));

      for(int k=1; k < N_; ++k)
      {

        Real x = Real(k) * l0_;

        Real y = 0;

        geom_.vertices_.push_back(Vector3<Real>(x,y,0));

        geom_.faces_.push_back(std::pair<int,int>(k-1,k));

        geom_.segments_.push_back(
            Segment3<Real>(geom_.vertices_[k-1], 
                           geom_.vertices_[k]
                            ));

        u_.push_back(Vec3(0,0,0));
        force_.push_back(Vec3(0,0,0));

        springs_.push_back(
            SimpleSpringConstraint<Real>(ks_, kd_, l0_,k-1,k,
                                         &geom_.vertices_[k-1],
                                         &geom_.vertices_[k],
                                         &u_[k-1],
                                         &u_[k]
                                         ));

      }

      Real kb = 16.0; 
      for(int k(0); k < N_-2; k++)
      {

        springs_.push_back(
            SimpleSpringConstraint<Real>(kb, kd_, 2.0*l0_,k,k+2,
                                         &geom_.vertices_[k],
                                         &geom_.vertices_[k+2],
                                         &u_[k],
                                         &u_[k+2]
                                         ));
      }

      for(auto &v: geom_.vertices_)
      {
        geom_.verticesOld_.push_back(v);
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

        Real m = 1.0;
        if(i < 5)
          m = 10000.0;

        Vec3 &pos = geom_.vertices_[i];
      
        vel = vel + dt_ * force * (1.0/m);

        pos = pos + dt_ * vel;

        force = Vec3(0,0,0);

        // At time 600 the stroke is reversed
        if(i >= N_ - nForce_)
        {
          pos.x = i * a0_; 
          vel.x = 0.0;
          vel.y = 0.0;
          if(strokeCount_ == 600)
          {
            vel.z = 0;
          }
        }

      }
    }; 
  };
}



#endif /* end of include guard: SOFTBODYINTERFACE_HPP_ZD18N5IP */
