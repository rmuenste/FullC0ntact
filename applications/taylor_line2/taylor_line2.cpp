#include <iostream>
#include <application.h>
#include <reader.h>
#include <paramline.h>
#include <softbody.hpp>
#include <mymath.h>
#include <algorithm>
#include <iterator>

#include <softbody.hpp>
#include <distancepointseg.h>

namespace i3d {

  template <typename T>
  struct SpringParameters {

    // Number of springs
    T N_;

    // Spring stiffness parameter for structural springs
    T ks_;
    
    // Spring stiffness parameter for bending springs
    T kb_;

    // Spring dampening parameter 
    T kd_;

    // Spring length parameters
    T a0_;
    T l0_;

  };

  template <typename T>
    struct SpringConfiguration {

      SpringConfiguration(int _N, T _kd, T _ks, T _a0, T _l0) : N(_N), kd(_kd), ks(_ks), kb(16.0), a0(_a0), l0(_l0)
      {

      }

      // Number of springs
      int N;

      // Spring stiffness parameter for structural springs
      T ks;
      
      // Spring stiffness parameter for bending springs
      T kb;

      // Spring dampening parameter 
      T kd;

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

        // Spring stiffness parameter for structural springs
        Real ks_;
        
        // Spring stiffness parameter for bending springs
        Real kb_;

        // Spring dampening parameter 
        Real kd_;

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

        SoftBody4() : N_(7), a0_(0.04), l0_(1.0*a0_), u_(N_), force_(N_), externalForce_(N_), ks_(10.0), kb_(16.0), kd_(-0.2)
        {
          transform_.setOrigin(Vec3(0, 0, 0));

          geom_.center_ = Vec3(0, 0, 0);

          geom_.vertices_.reserve(N_);

          velocity_ = Vec3(0, 0, 0);

          particleSize_ = 0.01;
        };

        SoftBody4(Real ks, Real kd) : N_(7), a0_(0.04), l0_(1.0*a0_), u_(N_), force_(N_), externalForce_(N_), ks_(ks), kb_(16.0), kd_(kd)
        {
          transform_.setOrigin(Vec3(0, 0, 0));

          geom_.center_ = Vec3(0, 0, 0);

          geom_.vertices_.reserve(N_);

          velocity_ = Vec3(0, 0, 0);

          particleSize_ = 0.01;
        };

        SoftBody4(int N, Real ks, Real kb, Real kd, Real ps) : N_(N), a0_(0.04),
        l0_(1.0*a0_), u_(N_), force_(N_), 
        externalForce_(N_), ks_(ks), kb_(kb), kd_(kd), particleSize_(ps)
        {
          transform_.setOrigin(Vec3(0, 0, 0));

          geom_.center_ = Vec3(0, 0, 0);

          geom_.vertices_.reserve(N_);

          velocity_ = Vec3(0, 0, 0);
        };

       /** 
        * Initialize a soft body
        * @param N Number of particles that make up the soft body
        * @param ks The linear stiffness spring constant 
        * @param kb The bending spring constant
        * @param kd The dampening constant
        * @param ps The radius of the particles
        */
        SoftBody4(int N, Real totalLength, Real ks, Real kb, Real kd, Real ps) : N_(N), a0_(0.04),
        l0_(1.0*a0_), u_(N_), force_(N_), 
        externalForce_(N_), ks_(ks), kb_(kb), kd_(kd), particleSize_(ps)
        {
          transform_.setOrigin(Vec3(0, 0, 0));

          geom_.center_ = Vec3(0, 0, 0);

          geom_.vertices_.reserve(N_);

          velocity_ = Vec3(0, 0, 0);

          a0_ = totalLength / Real(N-1);

          l0_ = a0_; 
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

          bool inConnection = false;
          for (int i(0); i <= geom_.vertices_.size() - 2; ++i)
          {
            Vec3 v0(geom_.vertices_[i].x,geom_.vertices_[i].y,0.0);  
            Vec3 v1(geom_.vertices_[i+1].x,geom_.vertices_[i+1].y,0.0);  
            //Segment3<Real> s(geom_.vertices_[i], geom_.vertices_[i + 1]);
            Segment3<Real> s(v0,v1);
            CDistancePointSeg<Real> distPointSeg(q, s);
            Real dist = distPointSeg.ComputeDistance();
            if (dist < 0.008)
            {
              id = 10;
              inConnection = true;
            }
          }

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

          Vec3 c(0.6, 0.16, 0);
          Real r = 0.05;

          Vec3 qq(q.x,q.y,0);

          if((c - qq).mag() < r)
          {
            id = 11;
            return true;
          }

          if (inConnection)
            return true;

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

          //externalForce();

          //windForce(t);

          //internalForce(t,it); 

          springForce();

          //integrate();

          //integrateMidpoint();

          integrateRK4();
        }

        void externalForce()
        {
//          for (int j(1); j < externalForce_.size(); ++j)
//          {
//            externalForce_[j] += myWorld.rigidBodies_[j]->force_;
//          }
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
            force.x += 0.1 * std::abs(std::sin(pos.x + t * 5.0) + 
                std::cos(pos.y + t * 5.0) / 3.0);
          }

        };

        void integrateRK4()
        {

          std::vector<Vector3<Real>> &u0 = u_; 
          std::vector<Vector3<Real>> &f0 = force_; 
          std::vector<Vector3<Real>> &fe = externalForce_; 

          std::vector< Vector3<Real> > xt;
          std::vector< Vector3<Real> > vt;
          std::vector< Vector3<Real> > ft;

          std::vector< Vector3<Real> > sumL(N_);
          std::vector< Vector3<Real> > sumK(N_);

          std::copy(geom_.vertices_.begin(), geom_.vertices_.end(), std::back_inserter(xt));

          std::copy(u0.begin(), u0.end(), std::back_inserter(vt));

          for (int j(0); j < N_; ++j) {
            ft.push_back(Vec3(0, 0, 0));
          }

          Real dt_half = 0.5 * dt_;

          // 1st RK step
          for(int i(1); i < N_; ++i)
          {
             Vec3 &vel = u0[i];

             Vec3 &force = f0[i];

             Real m = 0.008;
             if (i == 0) {
               m = 10000.0;
             }

             Vec3 g(0, -9.81, 0);

             Vec3 extForce = m * g;

             Vec3 totalForce = force + extForce;

             // l1
             ft[i] = totalForce * (1.0 / m);

             // k1
             // k1 is actually vt

             Vec3 &pos = geom_.vertices_[i];

             sumL[i] += dt_ / 6.0 * ft[i];

             sumK[i] += dt_ / 6.0 * vt[i];

             // k2: Needed for evaluation of l2
             // k2 = vt[i] + dt_half * ft[i];
             vel = vt[i] + dt_half * totalForce * (1.0/m);
             sumK[i] += dt_ / 3.0 * vel;

             // Needed for evaluation of l2
             // l2 = next F
             pos.x = xt[i].x + dt_half * vt[i].x;
             pos.y = xt[i].y + dt_half * vt[i].y;

             force = Vec3(0,0,0);
             extForce = Vec3(0,0,0);
          }

          springForce();

          for(int i(1); i < N_; ++i)
          {
             Vec3 &vel = u0[i];

             Vec3 &force = f0[i];

             Real m = 0.008;
             if (i == 0) {
               m = 10000.0;
             }

             Vec3 g(0, -9.81, 0);

             Vec3 extForce = m * g;

             // l2
             Vec3 totalForce = force + extForce;
             // l2
             ft[i] = totalForce * (1.0 / m);

             Vec3 &pos = geom_.vertices_[i];

             sumL[i] += dt_ / 3.0 * ft[i];

             // Needed for evaluation of l3
             // l3 = next F
             //vel = k2
             // for l3 we need to evaluate F at position xt + h/2 * k2
             pos.x = xt[i].x + dt_half * vel.x; 
             pos.y = xt[i].y + dt_half * vel.y; 

             // for k3
             // k3 = vt[i] + dt_half * ft[i] 
             vel = vt[i] + dt_half * ft[i];
             sumK[i] += dt_ / 3.0 * vel;

             force = Vec3(0,0,0);
             extForce = Vec3(0,0,0);
          }

          springForce();

          for(int i(1); i < N_; ++i)
          {
             Vec3 &vel = u0[i];

             Vec3 &force = f0[i];

             Real m = 0.008;
             if (i == 0) {
               m = 10000.0;
             }

             Vec3 g(0, -9.81, 0);

             Vec3 extForce = m * g;

             // l3
             Vec3 totalForce = force + extForce;
             // l3
             ft[i] = totalForce * (1.0 / m);

             sumL[i] += dt_ / 3.0 * ft[i];

             Vec3 &pos = geom_.vertices_[i];

             // Needed for evaluation of l4
             // l4 = next F
             //vel = k3
             // for l4 we need to evaluate F at position xt + h/2 * k3
             pos.x = xt[i].x + dt_ * vel.x; 
             pos.y = xt[i].y + dt_ * vel.y; 

             // for k4
             // k4 = vt[i] + dt_half * ft[i] 
             vel = vt[i] + dt_ * ft[i];
             sumK[i] += dt_ / 6.0 * vel;

             force = Vec3(0,0,0);
             extForce = Vec3(0,0,0);
          }

          springForce();

          for (int i(1); i < N_; ++i)
          {
             Vec3 &vel = u0[i];

             Vec3 &pos = geom_.vertices_[i];
             Vec3 &force = f0[i];

             Real m = 0.008;
             if (i == 0) {
               m = 10000.0;
             }

             Vec3 g(0, -9.81, 0);

             Vec3 extForce = m * g;

             // l4
             Vec3 totalForce = force + extForce;
             // l4
             ft[i] = totalForce * (1.0 / m);

             sumL[i] += dt_ / 6.0 * ft[i];

             Vec3 k4 = vel;

             vel = vt[i] + sumL[i];

             //k4 = vel
             pos.x = xt[i].x + sumK[i].x;
             pos.y = xt[i].y + sumK[i].y;
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

          for (int k(0); k < N_ - 2; k++)
          {
            springs_.push_back(
                SimpleSpringConstraint<Real>(kb_, kd_, 2.0*l0_, k, k + 2,
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

          for (int k(0); k < N_ - 2; k++)
          {
            springs_.push_back(
                SimpleSpringConstraint<Real>(kb_, kd_, 2.0*l0_, k, k + 2,
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

        void integrateMidpoint()
        {

          std::vector<Vector3<Real>> &u0 = u_; 
          std::vector<Vector3<Real>> &f0 = force_; 
          std::vector<Vector3<Real>> &fe = externalForce_; 

          std::vector< Vector3<Real> > x2;
          std::vector< Vector3<Real> > v2;
          std::vector< Vector3<Real> > f2;

          std::copy(geom_.vertices_.begin(), geom_.vertices_.end(), std::back_inserter(x2));

          std::copy(u0.begin(), u0.end(), std::back_inserter(v2));

          for (int j(0); j < N_; ++j)
            f2.push_back(Vec3(0, 0, 0));

          Real dt_half = 0.5 * dt_;

          for(int i(1); i < N_; ++i)
          {
            Vec3 &vel = u0[i];

            Vec3 &force = f0[i];

            Real m = 0.008;
            if (i == 0) {
              m = 10000.0;
            }

            Vec3 g(0, -9.81, 0);

            Vec3 extForce = m * g;

            Vec3 totalForce = force + extForce;

            f2[i] = totalForce * (1.0 / m);

            Vec3 &pos = geom_.vertices_[i];

            vel = vel + dt_half * totalForce * (1.0/m);

            pos.x = pos.x + dt_half * v2[i].x;
            pos.y = pos.y + dt_half * v2[i].y;

            force = Vec3(0,0,0);
            extForce = Vec3(0,0,0);
          }

          springForce();

          for(int i(1); i < N_; ++i)
          {
            Vec3 &vel = u0[i];

            Vec3 &force = f0[i];

            Real m = 0.008;
            if (i == 0) {
              m = 10000.0;
            }

            Vec3 g(0, -9.81, 0);

            Vec3 extForce = m * g;

            Vec3 totalForce = force + extForce;

            Vec3 &pos = geom_.vertices_[i];

            vel = v2[i] + dt_ * totalForce * (1.0/m);

            pos.x = x2[i].x + v2[i].x * dt_ + dt_ * dt_half * f2[i].x; 
            pos.y = x2[i].y + v2[i].y * dt_ + dt_ * dt_half * f2[i].y; 

            force = Vec3(0,0,0);
            extForce = Vec3(0,0,0);
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

            Real m = 0.008;
            if (i == 0) {
              m = 10000.0;
            }

            Vec3 g(0, -9.81, 0);

            Vec3 extForce = m * g;

            Vec3 totalForce = force + extForce;

            Vec3 &pos = geom_.vertices_[i];

            vel = vel + dt_ * totalForce * (1.0/m);

            pos.x = pos.x + dt_ * vel.x;
            pos.y = pos.y + dt_ * vel.y;

//            std::cout << " > Position[" << i << "]: " << pos;

//              std::cout << termcolor::bold << termcolor::white << myWorld.parInfo_.getId() <<  
//                " > Velocity[" << i << "]: " << vel << termcolor::reset;

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

        for (int k(0); k < sb.N_ - 2; k++)
        {
          sb.springs_.push_back(
              SimpleSpringConstraint<Real>(sb.kb_, sb.kd_, 2.0*sb.l0_, k, k + 2,
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
  


  class TaylorLineApp : public Application<> {
  public:

    std::shared_ptr<SoftBody4<Real, ParamLine<Real>>> softBodyPointer;
    SoftBody4<Real, ParamLine<Real> > *softBody_; 

    int steps_;
    Real dt_;
    Real time_;
    int step_;

    TaylorLineApp() : Application() {
      step_ = 0;
      time_ = 0.0;
      dt_ = 0.001;
    }

    void init(std::string fileName) {

      size_t pos = fileName.find(".");

      std::string ending = fileName.substr(pos);

      std::transform(ending.begin(), ending.end(), ending.begin(), ::tolower);
      if (ending == ".txt")
      {

        Reader myReader;
        //Get the name of the mesh file from the
        //configuration data file.
        myReader.readParameters(fileName, this->dataFileParams_);

      }//end if
      else if (ending == ".xml")
      {

        FileParserXML myReader;

        //Get the name of the mesh file from the
        //configuration data file.
        myReader.parseDataXML(this->dataFileParams_, fileName);

      }//end if
      else
      {
        std::cerr << "Invalid data file ending: " << ending << std::endl;
        std::exit(EXIT_FAILURE);
      }//end else

      dataFileParams_.rigidBodies_[0].kd_ = -1.0;
      std::cout << "> dataFileParams_.rigidBodies_[0].ks_ " << dataFileParams_.rigidBodies_[0].ks_ << std::endl;
      std::cout << "> dataFileParams_.rigidBodies_[0].kb_ " << dataFileParams_.rigidBodies_[0].kb_ << std::endl;
      std::cout << "> dataFileParams_.rigidBodies_[0].kd_ " << dataFileParams_.rigidBodies_[0].kd_ << std::endl;

      softBodyPointer = std::make_shared< SoftBody4<Real, ParamLine<Real>>>(
                        dataFileParams_.rigidBodies_[0].nSoftBodyParticles_, 0.32,
                        dataFileParams_.rigidBodies_[0].ks_, dataFileParams_.rigidBodies_[0].kb_,
                        dataFileParams_.rigidBodies_[0].kd_, 0.01);

      softBody_ = softBodyPointer.get();

      SpringConfiguration<Real> sc(softBody_->N_, softBody_->ks_,
                                   softBody_->kd_, softBody_->a0_, softBody_->l0_);
      
      InitSpringMesh2 initSprings(sc, *softBody_); 
      initSprings.init();

      softBody_->istep_ = 0;

      for(int i(0); i < softBody_->geom_.vertices_.size(); ++i)
      {
          std::cout << "Particle position: " << softBody_->geom_.vertices_[i];
      }

      std::cout << "> FC initialized " << std::endl;

    }

    void run()
    {
      for (int i(0); i < 10000; ++i) {
      //for (int i(0); i < 1; ++i) {
        std::cout << "Time: " << time_ << "|-----------------------|" << dt_ << "|it: " << step_ << std::endl;
        softBody_->step(time_, dt_, step_);
        if (step_ % 25 == 0) {
          CVtkWriter writer;
          std::ostringstream name;
          name << "output/line." << std::setfill('0') << std::setw(5) << step_ << ".vtk";
          writer.WriteParamLine(softBody_->geom_, softBody_->force_, name.str().c_str());
        }
        time_ += dt_;
        step_++;
      }
    }
  };

} //end namespace

using namespace i3d;

int main()
{
  TaylorLineApp myApp;
  myApp.init("start/sampleRigidBody.xml");
  myApp.run();
  
  return 0;
}
