#ifndef SOFTBODYINTERFACE_HPP_ZD18N5IP
#define SOFTBODYINTERFACE_HPP_ZD18N5IP

#include <softbody.hpp>
#include <distancepointseg.h>

namespace i3d {

  template <typename T>
  struct SpringParameters {

    // Number of springs
    T N_;

    // Spring stiffness parameters
    T kd_;
    T ks_;

    // Spring length parameters
    T a0_;
    T l0_;

  };

  template <typename T>
    struct SpringConfiguration {

      SpringConfiguration(int _N, T _kd, T _ks, T _a0, T _l0) : N(_N), kd(_kd), ks(_ks), a0(_a0), l0(_l0)
      {

      }

      // Number of springs
      int N;

      // Spring stiffness parameters
      T kd;
      T ks;

      // Spring length parameters
      T a0;
      T l0;

    };


  template<>
    class SoftBody4<Real, ParamLine<Real> > : public BasicSoftBody<Real, ParamLine<Real>>
    {
      public:

        int istep_;

        // Number of springs
        Real N_;

        Real m_;

        // Spring stiffness parameters
        Real kd_;
        Real ks_;

        // Spring length parameters
        Real a0_;
        Real l0_;

        Real dt_;

        // Physical parameters
        Real rho_;
        Real massAll_;
        Real volume_;

        Vec3 com_;

        Vec3 velocity_;

        Transformationr transform_;

        std::vector< SimpleSpringConstraint<Real> > springs_;

        // Solution vectors for velocity, force and external force 
        std::vector< Vector3<Real> > u_;
        std::vector< Vector3<Real> > force_;
        std::vector< Vector3<Real> > externalForce_;

        Real particleSize_;

        // Initializer functor
        // InitSpringMesh ftor_;

        SoftBody4() : N_(7), a0_(0.04), l0_(1.0*a0_), u_(N_), force_(N_), externalForce_(N_), ks_(10.0), kd_(-0.2)
      {

        transform_.setOrigin(Vec3(0, 0, 0));

        geom_.center_ = Vec3(0, 0, 0);

        geom_.vertices_.reserve(N_);

        velocity_ = Vec3(0, 0, 0);

        particleSize_ = 0.01;

      };


        SoftBody4(Real ks, Real kd) : N_(7), a0_(0.04), l0_(1.0*a0_), u_(N_), force_(N_), externalForce_(N_), ks_(ks), kd_(kd)
      {

        transform_.setOrigin(Vec3(0, 0, 0));

        geom_.center_ = Vec3(0, 0, 0);

        geom_.vertices_.reserve(N_);

        velocity_ = Vec3(0, 0, 0);

        particleSize_ = 0.01;

      };

        SoftBody4(int N, Real ks, Real kd, Real ps) : N_(N), a0_(0.04),
        l0_(1.0*a0_), u_(N_), force_(N_), 
        externalForce_(N_), ks_(ks), kd_(kd), particleSize_(ps)
      {
        transform_.setOrigin(Vec3(0, 0, 0));
        geom_.center_ = Vec3(0, 0, 0);
        geom_.vertices_.reserve(N_);
        velocity_ = Vec3(0, 0, 0);
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
          //Vec3 q = vQuery - transform_.getOrigin();
          Vec3 q(vQuery.x, vQuery.y, 0.0);
          for (int i(geom_.vertices_.size() - 1); i >= 0; --i)
          {

            Vec3 v(geom_.vertices_[i].x,geom_.vertices_[i].y,0.0);  
            //if ((geom_.vertices_[i] - q).mag() < particleSize_)
            if ((v - q).mag() < particleSize_)
            {
              id = i + 1;
              return true;
            }
          }

          for (int i(0); i <= geom_.vertices_.size() - 2; ++i)
          {
            Vec3 v0(geom_.vertices_[i].x,geom_.vertices_[i].y,0.0);  
            Vec3 v1(geom_.vertices_[i+1].x,geom_.vertices_[i+1].y,0.0);  
            //Segment3<Real> s(geom_.vertices_[i], geom_.vertices_[i + 1]);
            Segment3<Real> s(v0,v1);
            CDistancePointSeg<Real> distPointSeg(q, s);
            Real dist = distPointSeg.ComputeDistance();
            if (dist < 0.004)
            {
              id = 10;
              return true;
            }
          }

          Vec3 c(0.6, 0.16, 0);
          Real r = 0.05;

          Vec3 qq(q.x,q.y,0);

          if((c - qq).mag() < r)
          {
            id = 11;
            return true;
          }

          return false;
        }

        Vec3 getVelocity(const Vec3 &vQuery, int ind)
        {
          return u_[ind];
          Vec3 q = vQuery - transform_.getOrigin();
          int imin = 0;
          Real dmin = (geom_.vertices_[0] - q).mag();
          for (int i(1); i < geom_.vertices_.size(); ++i)
          {
            Real mmin = (geom_.vertices_[i] - q).mag();
            if (mmin < dmin)
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
          for (int i(0); i < geom_.vertices_.size(); ++i)
          {
            geom_.verticesOld_[i] = geom_.vertices_[i];
          }
        }

        void step(Real t, Real dt, int it)
        {
          dt_ = dt;

          externalForce();

          //windForce(t);

          //internalForce(t,it); 

          springForce();

          integrate();
        }

        void externalForce()
        {
          for (int j(1); j < externalForce_.size(); ++j)
          {
            externalForce_[j] += myWorld.rigidBodies_[j]->force_;
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

            if(myWorld.parInfo_.getId()==1)
            {
              std::cout << termcolor::bold << termcolor::green << myWorld.parInfo_.getId() <<  
                " > Spring[" << i << "]: " << f << termcolor::reset;
            }

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
            force.x += 0.1 * std::abs(std::sin(pos.x + t * 5.0) + 
                std::cos(pos.y + t * 5.0) / 3.0);
          }

        };

        void init2()
        {

          //ftor_.sb = this;
          //ftor_();

          Real xx = 0.65;
          Real yy = 0.16;

          geom_.vertices_.push_back(
              Vector3<Real>(xx,
                yy,
                0));

          for (int k = 1; k < N_; ++k)
          {

            Real x = xx + (Real(k) * l0_);

            Real y = yy;

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

        void init()
        {

          Real xx = 0.796354;
          Real yy = 0.4;

          geom_.vertices_.push_back(
              Vector3<Real>(xx,
                yy,
                0));

          //          u_.push_back(Vec3(0, 0, 0));
          //
          //          force_.push_back(Vec3(0, 0, 0));
          //          externalForce_.push_back(Vec3(0, 0, 0));

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

            //            u_.push_back(Vec3(0, 0, 0));
            //
            //            force_.push_back(Vec3(0, 0, 0));
            //
            //            externalForce_.push_back(Vec3(0, 0, 0));

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

          for(int i(1); i < N_; ++i)
          {
            Vec3 &vel = u0[i];

            Vec3 &force = f0[i];

            Vec3 &extForce = fe[i];

            Vec3 totalForce = force + extForce;

            if(myWorld.parInfo_.getId()==1)
            {

              std::cout << termcolor::bold << termcolor::cyan << myWorld.parInfo_.getId() <<  
                " > Spring force[" << i << "]: " << force << termcolor::reset;
              std::cout << termcolor::bold << termcolor::magenta << myWorld.parInfo_.getId() <<  
                " > Fluid force[" << i << "]: " << extForce << termcolor::reset;
              std::cout << termcolor::bold << termcolor::red << myWorld.parInfo_.getId() <<  
                " > Total force[" << i << "]: " << totalForce << termcolor::reset;

            }

            Real m = 0.008;
            if(i == 0)
              m = 10000.0;

            Vec3 g(0,0,0);

            Vec3 &pos = geom_.vertices_[i];

            vel.x = vel.x + dt_ * totalForce.x * (1.0/m);
            vel.y = vel.y + dt_ * totalForce.y * (1.0/m) + dt_ * g.y;

            pos.x = pos.x + dt_ * vel.x;
            pos.y = pos.y + dt_ * vel.y;

            if(myWorld.parInfo_.getId()==1)
            {

              std::cout << termcolor::bold << termcolor::white << myWorld.parInfo_.getId() <<  
                " > Position[" << i << "]: " << pos << termcolor::reset;
              std::cout << termcolor::bold << termcolor::white << myWorld.parInfo_.getId() <<  
                " > Velocity[" << i << "]: " << vel << termcolor::reset;
            }

            force = Vec3(0,0,0);
            extForce = Vec3(0,0,0);
          }
        }; 
    };

  class InitSpringMesh2 {

    public:

      SpringConfiguration<Real> &springConf;

      SoftBody4<Real, ParamLine<Real> > &sb;


      InitSpringMesh2(SpringConfiguration<Real> &_springConf, 
          SoftBody4<Real, ParamLine<Real> > &_sb) : 
        springConf(_springConf), sb(_sb) 
      {

      };

      void init()
      {

        Real xx = 0.65;
        Real yy = 0.16;

        sb.geom_.vertices_.push_back(
            Vector3<Real>(xx,
              yy,
              0));

        for (int k = 1; k < sb.N_; ++k)
        {

          Real x = xx + (Real(k) * sb.l0_);

          Real y = yy;

          sb.geom_.vertices_.push_back(Vector3<Real>(x, y, 0));

          sb.geom_.faces_.push_back(std::pair<int, int>(k - 1, k));

          sb.geom_.segments_.push_back(
              Segment3<Real>(sb.geom_.vertices_[k - 1],
                sb.geom_.vertices_[k]
                ));

          sb.springs_.push_back(
              SimpleSpringConstraint<Real>(sb.ks_, sb.kd_, sb.l0_, k - 1, k,
                &sb.geom_.vertices_[k - 1],
                &sb.geom_.vertices_[k],
                &sb.u_[k - 1],
                &sb.u_[k]
                ));

        }

        Real kb = 16.0;
        for (int k(0); k < sb.N_ - 2; k++)
        {
          sb.springs_.push_back(
              SimpleSpringConstraint<Real>(kb, sb.kd_, 2.0*sb.l0_, k, k + 2,
                &sb.geom_.vertices_[k],
                &sb.geom_.vertices_[k + 2],
                &sb.u_[k],
                &sb.u_[k + 2]
                ));
        }

        for (auto &v : sb.geom_.vertices_)
        {
          sb.geom_.verticesOld_.push_back(v);
        }

      }

  }; // class InitSpringMesh2 
  

} //end namespace

#endif /* end of include guard: SOFTBODYINTERFACE_HPP_ZD18N5IP */
