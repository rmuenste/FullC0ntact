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

  template <typename T>
    class DistanceConstraint {
      public:
        int p1, p2;
        T rest_length;
        T k_stiff;
        T k_prime;

        DistanceConstraint() {

        }

        ~DistanceConstraint() {

        }

        DistanceConstraint(int _p1, int _p2, T _rest_length, T _k, T _k_prime) : p1(_p1), p2(_p2), rest_length(_rest_length), k_stiff(_k), k_prime(_k_prime) {

        }
    };

  template <typename T>
    class BendingConstraint {
      public:
        int p1, p2, p3;
        T phi0;
        T k_bend;
        T k_prime;

        BendingConstraint() {

        }

        ~BendingConstraint() {

        }

        BendingConstraint(int _p1, int _p2, int _p3, T _phi0, T _k, T _k_prime) : p1(_p1), p2(_p2), p3(_p3), phi0(_phi0), k_bend(_k), k_prime(_k_prime) {

        }
    };

  template<>
    class SoftBody5<Real, ParamLine<Real> > : public BasicSoftBody<Real, ParamLine<Real>>
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

        Real m = 0.008;

        Real w = 1.0 / m;

        Transformationr transform_;

        std::vector< DistanceConstraint<Real> > distanceConstraints_;
        std::vector< BendingConstraint<Real> > bendingConstraints_;

        // Solution vectors for velocity, force and external force 
        std::vector< Vector3<Real> > u_;
        std::vector< Vector3<Real> > force_;
        std::vector< Vector3<Real> > externalForce_;

        std::vector< Vector3<Real> > positionEstimate_;

        Real particleSize_;

        SoftBody5() : N_(7), a0_(0.04), l0_(1.0*a0_), u_(N_), force_(N_), externalForce_(N_), ks_(10.0), kb_(16.0), kd_(-0.2)
        {
          transform_.setOrigin(Vec3(0, 0, 0));

          geom_.center_ = Vec3(0, 0, 0);

          geom_.vertices_.reserve(N_);

          velocity_ = Vec3(0, 0, 0);

          particleSize_ = 0.01;
        };

       /** 
        * Initialize a soft body
        * @param N Number of particles that make up the soft body
        * @param ks The linear stiffness spring constant 
        * @param kb The bending spring constant
        * @param kd The dampening constant
        * @param ps The radius of the particles
        */
        SoftBody5(int N, Real totalLength, Real ks, Real kb, Real kd, Real ps) : N_(N), a0_(0.04),
        l0_(1.0*a0_), u_(N_), force_(N_), 
        externalForce_(N_), positionEstimate_(N_), ks_(ks), kb_(kb), kd_(kd), particleSize_(ps)
        {
          transform_.setOrigin(Vec3(0, 0, 0));

          geom_.center_ = Vec3(0, 0, 0);

          geom_.vertices_.reserve(N_);

          velocity_ = Vec3(0, 0, 0);

          a0_ = totalLength / Real(N-1);

          l0_ = a0_; 
        };

        virtual ~SoftBody5(){};

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

        Real getDihedralAngle(int p1, int p2, int p3) {
          Vec3 v1 = geom_.vertices_[p1];
          Vec3 v2 = geom_.vertices_[p2];
          Vec3 v3 = geom_.vertices_[p3];

          Vec3 n1(v1 - v2);
          Vec3 n2(v2 - v3);

          n1.Normalize();
          n2.Normalize();

          return std::acos(n1 * n2);
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

        void step(Real t, Real dt)
        {
          dt_ = dt;

          computeForce();

          integrate();

          updateInternalConstraints();

          integrateVerlet(); 
        }

        void integrateVerlet() {

          std::vector<Vector3<Real>> &u0 = u_; 
          std::vector<Vector3<Real>> &f0 = force_; 
          std::vector<Vector3<Real>> &fe = externalForce_; 

          for(int i(1); i < N_; ++i)
          {
            Vec3 &vel = u0[i];

            Vec3 &force = f0[i];

            Vec3 &pos = geom_.vertices_[i];

            vel = (positionEstimate_[i] - pos)/dt_;
           
            pos = positionEstimate_[i]; 

            force = Vec3(0,0,0);
          }

        }

        void updateBendingConstraint(unsigned i) {
          int p1 = bendingConstraints_[i].p1;
          int p2 = bendingConstraints_[i].p2;
          int p3 = bendingConstraints_[i].p3;

          Vec3 v1 = geom_.vertices_[p1];
          Vec3 v2 = geom_.vertices_[p2];
          Vec3 v3 = geom_.vertices_[p3];

          Vec3 n1(v1 - v2);
          Vec3 n2(v2 - v3);

          n1.Normalize();
          n2.Normalize();

          Real d = n1 * n2;

          if(d < -1.0) {
            d = -1.0;
          } else if(d > 1.0) {
            d = 1.0;
          }

          Real phi = std::acos(d);

        }

        void updateDistanceConstraint(unsigned i) {

          int p1 = distanceConstraints_[i].p1;
          int p2 = distanceConstraints_[i].p2;

          // This is actually the directional derivative of the distance constraint
          Vec3 dir = geom_.vertices_[p1] - geom_.vertices_[p2];

          Real length = dir.mag();

          Real w1 = w, w2 = w;

          Real invMass = w1 + w2;

          // The variable dp is the correction of the distance constraint
          // The last factor <XXX * distanceConstraints_[i].k_prime> is the multiplication with the 
          // stiffness of the constraint
          Vec3 dp = (1.0 / invMass) * (length - distanceConstraints_[i].rest_length) * (dir / length) * distanceConstraints_[i].k_prime;

          // Update the positions with the distance correction
          if (p1 != 0)
            positionEstimate_[p1] -= dp * w1;

          if (p2 != 0)
            positionEstimate_[p2] += dp * w2;

        }

        void updateInternalConstraints()
        {
           unsigned solverIter = 2;
           for (unsigned i(0); i < solverIter; ++i) {
             for (unsigned j(0); j < distanceConstraints_.size(); ++j) {
               updateDistanceConstraint(j);
             }
//             for (unsigned j(0); j < bendingConstraints_.size(); ++j) {
//               updateBendingConstraint(j);
//             }
           }
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

        void computeForce()
        {
          Vec3 g(0, -9.81, 0);

          for (int k = 1; k < N_; ++k)
          {
            Vec3 &force = force_[k];

            force = g;

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

        void integrate()
        {

          std::vector<Vector3<Real>> &u0 = u_; 
          std::vector<Vector3<Real>> &f0 = force_; 
          std::vector<Vector3<Real>> &fe = externalForce_; 

          for(int i(1); i < N_; ++i)
          {
            Vec3 &vel = u0[i];

            Vec3 &force = f0[i];

            Vec3 &pos = geom_.vertices_[i];
            positionEstimate_[i] = geom_.vertices_[i];
            Vec3 &posEst = positionEstimate_[i];

            // Apply dampening
            vel *= 0.99;

            vel = vel + dt_ * force * w;
           
            posEst.x = pos.x + dt_ * vel.x;
            posEst.y = pos.y + dt_ * vel.y;

//            std::cout << " > Position[" << i << "]: " << pos;

//              std::cout << termcolor::bold << termcolor::white << myWorld.parInfo_.getId() <<  
//                " > Velocity[" << i << "]: " << vel << termcolor::reset;

            force = Vec3(0,0,0);
          }
        }; 
    };

  class InitSpringMesh2 {

    public:

      SpringConfiguration<Real> &springConf;

      SoftBody5<Real, ParamLine<Real> > &sb;


      InitSpringMesh2(SpringConfiguration<Real> &_springConf, 
          SoftBody5<Real, ParamLine<Real> > &_sb) : 
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

          Real _k = 0.5;
          Real k_prime = 1.0 - std::pow(1.0 - _k, 1.0 / 2.0);
          sb.distanceConstraints_.push_back(
            DistanceConstraint<Real>(k - 1, k, sb.l0_, _k, k_prime)
          );

        }

        Real _k = 0.2;
        Real k_prime = 1.0 - std::pow(1.0 - _k, 1.0 / 2.0);
        for (int k(0); k < sb.N_ - 2; k++)
        {
          sb.distanceConstraints_.push_back(
            DistanceConstraint<Real>(k, k + 2, 2.0 * sb.l0_, _k, k_prime));
//          Real phi0 = sb.getDihedralAngle(k, k+1, k+2);
//          sb.bendingConstraints_.push_back(
//              BendingConstraint<Real>(k, k+1, k+2, phi0, _k, k_prime)
//              );
        }

        for (auto &v : sb.geom_.vertices_)
        {
          sb.geom_.verticesOld_.push_back(v);
        }

      }

  }; // class InitSpringMesh2 

  class PositionBasedDynamicsApp : public Application<> {
  public:

    std::shared_ptr<SoftBody5<Real, ParamLine<Real>>> softBodyPointer;
    SoftBody5<Real, ParamLine<Real> > *softBody_; 

    int steps_;
    Real dt_;
    Real time_;
    int step_;

    PositionBasedDynamicsApp() : Application() {
      step_ = 0;
      time_ = 0.0;
      dt_ = 0.0001;
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
      std::cout << "> Number of SB particles " << dataFileParams_.rigidBodies_[0].nSoftBodyParticles_ << std::endl;
      std::cout << "> dataFileParams_.rigidBodies_[0].ks_ " << dataFileParams_.rigidBodies_[0].ks_ << std::endl;
      std::cout << "> dataFileParams_.rigidBodies_[0].kb_ " << dataFileParams_.rigidBodies_[0].kb_ << std::endl;
      std::cout << "> dataFileParams_.rigidBodies_[0].kd_ " << dataFileParams_.rigidBodies_[0].kd_ << std::endl;

      softBodyPointer = std::make_shared< SoftBody5<Real, ParamLine<Real>>>(
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
        std::cout << "Time: " << time_ << "|-----------------------|" << dt_ << "|it: " << step_ << std::endl;
        softBody_->step(time_, dt_);
        if (step_ % 250 == 0) {
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
  PositionBasedDynamicsApp myApp;
  myApp.init("start/sampleRigidBody.xml");
  myApp.run();
  
  return 0;
}
