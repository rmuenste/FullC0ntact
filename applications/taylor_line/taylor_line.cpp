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

    Real A_, q_, N_, f_, m_, kd_, ks_;

    Real a0_, l0_, dt_, rho_, massAll_;

    Real volume_;

    Vec3 com_;

    Vec3 velocity_;

    Transformationr transform_;

    std::vector< SimpleSpringConstraint<Real> > springs_;

    std::vector< Vector3<Real> > u_;

    std::vector< Vector3<Real> > force_;

    std::vector<Real> L_;

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

    void init()
    {

      ks_ = 2000.0;
      kd_ = -0.2;

      Real pi = CMath<Real>::SYS_PI;

      Real t = 0.0;

      Real frq = 100.0;

      // The first vertex of the taylor line is (0,0,0)
      geom_.vertices_.push_back(Vector3<Real>(0, 0, 0));

      u_.push_back(Vec3(0, 0, 0));
      force_.push_back(Vec3(0, 0, 0));

      Real l_c = 24.75;

      Real b = 0.15;
      // generate a start vector
      Vec3 t_0(0.0,0.5,0.0);

      // scale the vector to get the desired length
      //t_0 *= 0.8836;
      
      // Generate the initial shape
      for (int k = 0; k < N_-1; ++k)
      {
        Real x = Real(k) * l0_;

        Real y = b * std::sin(2.0 * pi * (frq * t + x / (l_c)));

        // Get a rotation matrix
        Mat3 r;
        r.MatrixFromAngles( Vec3(0,0,l0_ * y));

        // Multiply the vector by the rotation matrix
        t_0 = r * t_0;

        geom_.vertices_.push_back(geom_.vertices_[k] + t_0);
     
      }

      Vec3 e = getVectorE();

      Vec3 d(1.0, 0, 0);

      Real beta = std::acos(e * d);

      Mat3 r;
        r.MatrixFromAngles( Vec3(0,0,-beta));

      Vec3 p_0(0.0,0.5,0.0);
 
      Vec3 s_0 = r * p_0;

      geom_.vertices_[0] = Vec3(0,0,0);

      geom_.vertices_[1] = s_0;
      

      // Generate the initial shape
      for (int k = 2; k < N_; ++k)
      {
        Real x = Real(k) * l0_;

        Real y = b * std::sin(2.0 * pi * (frq * t + x / (l_c)));

        // Get a rotation matrix
        Mat3 r;
        r.MatrixFromAngles( Vec3(0,0,l0_ * y));

        // Multiply the vector by the rotation matrix
        s_0 = r * s_0;

        geom_.vertices_[k] = (geom_.vertices_[k-1] + s_0);
     
      }


      // Generate the geometry for ParaView output
      for (int k = 1; k < N_; ++k)
      {
        geom_.faces_.push_back(std::pair<int, int>(k - 1, k));

        geom_.segments_.push_back(Segment3<Real>(geom_.vertices_[k - 1],
          geom_.vertices_[k]
          ));

        // Fill up the std::vectors for velocity u_ and force force_
        u_.push_back(Vec3(0, 0, 0));

        force_.push_back(Vec3(0, 0, 0));
      }

      // Calculate the length of the individual segments
      for (int i(1); i < N_; ++i)
      {
        Vec3 t_i = geom_.vertices_[i] - geom_.vertices_[i - 1];
        Real l_i = t_i.mag();
        L_.push_back(l_i);
      }

      // Add the springs
      for (int k = 1; k < N_; ++k)
      {
        Vec3 t_i = geom_.vertices_[k] - geom_.vertices_[k - 1];
        Real l_i = t_i.mag();
        springs_.push_back(
          SimpleSpringConstraint<Real>(ks_, kd_, l_i, k - 1, k,
            &geom_.vertices_[k - 1],
            &geom_.vertices_[k],
            &u_[k - 1],
            &u_[k]
            ));
      }

    } 

    // The contour length is the sum of the 
    // magnitude of all connecting edges
    Real getContourLength()
    {
      Real contourLength = 0.0;
      for(int i(1); i < N_; ++i) 
      {
        contourLength += (geom_.vertices_[i] - geom_.vertices_[i-1]).mag(); 
      }
      return contourLength;
    }

    // The contour length is the sum of the 
    // magnitude of all connecting edges
    Real getWavelengthContour()
    {
      Real l_c = 0.0;
      for(int i(1); i < 50; ++i) 
      {
        l_c += (geom_.vertices_[i] - geom_.vertices_[i-1]).mag(); 
      }
      l_c += 0.5 * (geom_.vertices_[50] - geom_.vertices_[49]).mag(); 
      return l_c;
    }

    Vec3 getVectorE()
    {
      Vec3 e(0, 0, 0);
      for(int i(1); i < N_; ++i) 
      {
        e += (geom_.vertices_[i] - geom_.vertices_[i-1]); 
      }
      e.normalize();
      return e;
    }

    Real getEnd2EndDistance()
    {
      Vec3 e = getVectorE();

      Real s0 = e * geom_.vertices_[0];
      Real s1 = e * geom_.vertices_[N_-1];

      return std::abs(s1 - s0);
    }

    Real getAmplitude()
    {
      Real A = 0.0;
      Vec3 e = getVectorE();

      Mat3 r;
      Real pi = CMath<Real>::SYS_PI;
      r.MatrixFromAngles( Vec3(0,0,0.5 * pi));
      e = r * e;
      for(int i(1); i < N_; ++i) 
      {
        Real B = std::abs(e * geom_.vertices_[i]);
        if (A < B)
        {
          A = B;
        }
      }

      return A;
    }

    Vec3 getcenterofmass()
    {
      Vec3 r_(0,0,0);
      for(int i(0); i < N_; ++i)
      {
       r_ += geom_.vertices_[i];
      }
      r_ *= 1.0 / N_;
      return r_;
    }

    void getAngles()
    {

      Vec3 t0(0, 0, 0);
      Vec3 e(0, 0, 0);
      t0 = (geom_.vertices_[1] - geom_.vertices_[0]); 

      for(int i(2); i < N_; ++i) 
      {
        e = (geom_.vertices_[i] - geom_.vertices_[i-1]); 
        t0.normalize();
        e.normalize();
        Real angle = std::acos(t0 * e);
        std::cout << "> Angle[" << i-1 << "]: " << angle << std::endl;
        t0 = e;
      }

      e.normalize();
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
      
      for(unsigned i(0); i < springs_.size(); ++i)
      {
        SimpleSpringConstraint<Real> &spring = springs_[i];

        Vector3<Real> f = spring.evalForce();

        force_[spring.i0_] += f;
        force_[spring.i1_] -= f;
      }

      Real L = getContourLength();
      Real p = 500 * L; //p=10 * L
      Real kappa = p;

      Real pi = CMath<Real>::SYS_PI;

      // 1 second wave propagation time 
      f_ = 0.1/10.0;     //f=1.0/10
      A_ = 0.3;          //A=0.2
      std::vector<Real> alphas;
      //Real lambda_c = (2.0 * pi)/(l0_ * N_);
   
      Real lambda_c = getWavelengthContour(); 
      
      for(int i(0); i < N_; ++i)
      {
        Real xx = (i+1) * l0_;
        Real c_nt = A_ * std::sin(2.0 * pi * f_ * t + xx * 2.0 * pi * 1.0/lambda_c);
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

        force_[j]+=force_b;


      }

      // handle case 0
      Mat3 r;

      Vec3 t_0 = geom_.vertices_[1] - geom_.vertices_[0]; 
      Vec3 t_1 = geom_.vertices_[2] - geom_.vertices_[1]; 
      r.MatrixFromAngles( Vec3(0,0,alphas[0]));
      Mat3 rt = r.GetTransposedMatrix();
      Vec3 term3 = t_1 - (rt * t_1);
      force_[0]+=kappa*term3;
      
      // handle case 1
      r.MatrixFromAngles( Vec3(0,0,alphas[0]));
      rt = r.GetTransposedMatrix();

      Vec3 term2 = t_1 - t_0 + (rt * t_1) - (r * t_0);

      r.MatrixFromAngles( Vec3(0,0,alphas[1]));
      rt = r.GetTransposedMatrix();
      term3 = t_1 - (rt * t_1);
      force_[1]+=kappa*(term2+term3);

      // handle case 98
      
      Vec3 t_96 = geom_.vertices_[97] - geom_.vertices_[96];
      Vec3 t_97 = geom_.vertices_[98] - geom_.vertices_[97];
      Vec3 t_98 = geom_.vertices_[99] - geom_.vertices_[98];

      r.MatrixFromAngles( Vec3(0,0,alphas[96]));
      Vec3 term1 = (r * t_96) - t_97;
      r.MatrixFromAngles( Vec3(0,0,alphas[97]));
      rt = r.GetTransposedMatrix();
    
       term2 = t_98 - t_97 + (rt * t_98) - (r * t_97);
      //force_[98]+=kappa*(term1+term2);

      // handle case 99
       t_97 = geom_.vertices_[98] - geom_.vertices_[97]; 
       t_98 = geom_.vertices_[99] - geom_.vertices_[98]; 
      r.MatrixFromAngles( Vec3(0,0,alphas[97]));
       term1 = (r * t_97) - t_98;
      force_[99]+=kappa*term1;

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

        Vec3 &pos = geom_.vertices_[i];
      
        vel = vel + dt_ * force * (1.0/m);

        // add some dampening
        vel *= 0.98;

        pos = pos + dt_ * vel;

      }
    }; 


    //void integrateverlet()
    //{

      //std::vector<Vector3<Real>> &u0 = u_; 

      //std::vector<Vector3<Real>> &f0 = force_; 

      //for(int i(0); i < N_; ++i)
      //{
        //Vec3 &vel = u0[i];

        //Vec3 &force = f0[i];

        //Real m = 1.0;

        //Vec3 &pos = geom_.vertices_[i];

        //Vec3 posold1 = geom_.vertices_[i];
      
        //vel = vel + dt_ * force * (1.0/m);

        // add some dampening
        //vel *= 0.98;

        //pos = 2 * pos -posold_i + dt_ * dt_* force * (1.0/m) ;

        //Vec3 posold_i = posold1;


      //}
    //}; 

    void step(Real t, Real dt, int it)
    {
      dt_ = dt;

      internalForce(t); 
      
      integrate();
    }

  };

  class TaylorLineApp : public Application {
  public:
    SoftBody4<Real, ParamLine<Real>> taylorLine_;
    int steps_;
    Real dt_;
    Real time_;
    int step_;

    TaylorLineApp() : Application() {

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
      taylorLine_.init();

      steps_ = 200000;

      dt_ = 0.0001;

      time_ = 0;

      step_ = 0;

      Vec3 r0 = taylorLine_.getcenterofmass();

      Vec3 rold = taylorLine_.getcenterofmass();

      for(int istep(0); istep <= steps_; ++istep)
      {
        Vec3 e = taylorLine_.getVectorE();
        Vec3 r = taylorLine_.getcenterofmass();
        Real v_ = ((r-rold) * e) / dt_;
        rold = r;

        std::cout << "Time: " << time_ << "|-----------------------|" << dt_ << "|it: " << istep<< std::endl;
 
        std::cout << "Velocity: " << v_  << istep<< std::endl;

        taylorLine_.step(time_,dt_, istep);

        CVtkWriter writer;
        std::ostringstream name;
        name << "output/line." << std::setfill('0') << std::setw(5) << istep << ".vtk";

        if(istep%100==0)
        writer.WriteParamLine(taylorLine_.geom_, taylorLine_.force_, name.str().c_str());
        time_ += dt_;
        step_++;

        std::cout << "Contour length of the swimmer: " << taylorLine_.getContourLength() << std::endl;

        std::cout << "e: " << taylorLine_.getVectorE();

        std::cout << "Amplitude: " << taylorLine_.getAmplitude() << std::endl;
        taylorLine_.getEnd2EndDistance();
        std::cout << "End2end distance: " << taylorLine_.getEnd2EndDistance() << std::endl;

        Real lc = taylorLine_.getWavelengthContour();
        std::cout << "lambda contour: " << lc << std::endl;
        std::cout << "lambda contour2: " << 0.5 * taylorLine_.getContourLength() << std::endl;
        
       
        
      }
       Vec3 e = taylorLine_.getVectorE();
       Vec3 r = taylorLine_.getcenterofmass();
       Real vall_ = ((r - r0) * e) /(steps_* dt_);
       
       std::cout << "Velocityall: " << vall_  << std::endl;
       
    }
  };
}

using namespace i3d;

int main()
{

  TaylorLineApp myApp;
  myApp.init("start/sampleRigidBody.xml");
  myApp.run();
  
  return 0;
}
