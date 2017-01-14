#ifndef SOFTBODY_HPP_NRX1JCK5
#define SOFTBODY_HPP_NRX1JCK5

#include <vector3.h>
#include <segment3.h>
#include <paramline.h>
#include <springconstraint.hpp>
#include <transform.h>
#include <cmath>

namespace i3d
{

  template <typename T, class G> 
  class BasicSoftBody
  {
  public:
    BasicSoftBody ()
    {

    };

    virtual ~BasicSoftBody ()
    {

    };

    G geom_;

    virtual void internalForce(T t){}; 

    virtual void applyForce(T dt){}; 

    virtual void init(){}; 

    virtual void integrate(){}; 
    
  
  private:
    /* data */
  };

  template<typename T, class G>
  class SoftBody : public BasicSoftBody<T, G>
  {
  public:
    SoftBody ()
    {

    };

    virtual ~SoftBody ()
    {

    };

    void internalForce(T t){}; 

    void applyForce(T dt){}; 

    void init(){}; 

    void integrate(){}; 
  
  private:
    /* data */
  };

  template<>
  class SoftBody<Real, ParamLine<Real>[2] > : public BasicSoftBody<Real, ParamLine<Real>[2]>
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

    int n_mid_;

    int n_head_;

    int offset_;

    std::vector< BasicSpringConstraint<Real> > springs_;

    std::vector< BasicSpringConstraint<Real> > bendSprings_;

    std::vector< Vector3<Real> > u_[2];

    std::vector< Vector3<Real> > force_[2];

    SoftBody () : A_(3.2), N_(100),  f_(1.0/200.0), a0_(0.5), l0_(1.0*a0_)
    {

      dt_ = 0.001;
      n_mid_ = 14;
      n_head_ = 25;

    };

    virtual ~SoftBody (){};

    void internalForce(Real t)
    {

      const double pi = 3.1415926535897;

      Real q = 4.0 * pi/(l0_ * N_); 

      int j = 0;

      std::vector<Vector3<Real>> &u0 = u_[0]; 
      std::vector<Vector3<Real>> &f0 = force_[0]; 

      for(int i(n_mid_); i < N_; ++i)
      {

        Real x = Real(j) * a0_;

        Real xl = -2.0 * pi * f_ * t + q * x;

        Real y = A_ * std::sin(xl);

        geom_[0].vertices_[i].x  = x;
        geom_[0].vertices_[i].y  = y;

        j++;

      }

      // evaluate force for the first 14 segments
      for(int i(0); i <= n_mid_-1; ++i)
      {
        BasicSpringConstraint<Real> &spring = springs_[i];

        Vector3<Real> f = spring.evalForce(geom_[0].vertices_[spring.i0_.first],
                                           geom_[0].vertices_[spring.i1_.first],
                                           u0[spring.i0_.first],
                                           u0[spring.i1_.first]
                                           );

        //std::cout << "f: " << f << std::endl;

        f0[spring.i0_.first] += f;
        f0[spring.i1_.first] -= f;
      }

      for(unsigned i(0); i < bendSprings_.size(); ++i)
      {
        BasicSpringConstraint<Real> &spring = bendSprings_[i];
        int g0 = spring.i0_.second;
        int g1 = spring.i1_.second;

        Vector3<Real> f = spring.evalForce(geom_[g0].vertices_[spring.i0_.first],
                                           geom_[g1].vertices_[spring.i1_.first],
                                           u_[g0][spring.i0_.first],
                                           u_[g1][spring.i1_.first]
                                           );

        //std::cout << "f: " << f << std::endl;

        force_[g0][spring.i0_.first] += f;
        force_[g1][spring.i1_.first] -= f;
      }

      for(unsigned i(offset_); i < springs_.size(); ++i)
      {
        BasicSpringConstraint<Real> &spring = springs_[i];

        int g0 = spring.i0_.second;
        int g1 = spring.i1_.second;

        Vector3<Real> f = spring.evalForce(geom_[g0].vertices_[spring.i0_.first],
                                           geom_[g1].vertices_[spring.i1_.first],
                                           u_[g0][spring.i0_.first],
                                           u_[g1][spring.i1_.first]
                                           );

        //std::cout << "f: " << f << std::endl;

        force_[g0][spring.i0_.first] += f;
        force_[g1][spring.i1_.first] -= f;
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

      const double pi = 3.1415926535897;

      Real q = 4.0 * pi/(l0_ * N_); 

      Real xx = Real(-n_mid_) * l0_;
      Real yy = A_ * std::sin(-2.0 * pi * f_ + q * xx);


      geom_[0].vertices_.push_back(Vector3<Real>(xx,
                                                 yy,
                                                 0));

      std::vector<Vector3<Real>> &u0 = u_[0]; 
      std::vector<Vector3<Real>> &f0 = force_[0]; 

      std::vector<Vector3<Real>> &u1 = u_[1]; 
      std::vector<Vector3<Real>> &f1 = force_[1]; 

      u0.push_back(Vector3<Real>(0,0,0));

      f0.push_back(Vector3<Real>(0,0,0));

      ks_ = 0.8;

      kd_ = -0.2;

      int j = -13; 

      for(int k=1; k < N_; ++k)
      {

        Real x = Real(j) * l0_;

        Real y = A_ * std::sin(-2.0 * pi * f_ + q * x);

        geom_[0].vertices_.push_back(Vector3<Real>(x,y,0));
        u0.push_back(Vector3<Real>(0,0,0));

        f0.push_back(Vector3<Real>(0,0,0));

        geom_[0].faces_.push_back(std::pair<int,int>(k-1,k));

        geom_[0].segments_.push_back(Segment3<Real>(geom_[0].vertices_[k-1], 
                                                    geom_[0].vertices_[k]
                                                   ));

        std::pair<int, int> index0(k-1,0);
        std::pair<int, int> index1(k,0);


        springs_.push_back(BasicSpringConstraint<Real>(ks_, kd_, l0_, index0, index1,
                                                       &geom_[0].vertices_[k-1],
                                                       &geom_[0].vertices_[k],
                                                       &u0[k-1],
                                                       &u0[k]
                                                       ));

        j++;

      }

      for(int k(0); k < n_mid_; k++)
      {

        std::pair<int, int> index0(k,0);
        std::pair<int, int> index1(k+2,0);

        bendSprings_.push_back(
            BasicSpringConstraint<Real>(0.85, -0.25, 
                                        2.0*l0_, index0, index1,
                                        &geom_[0].vertices_[k],
                                        &geom_[0].vertices_[k+2],
                                        &u0[k],
                                        &u0[k+2]
                                       ));

      }

      Real t = 0.0;

      for(int i(0); i < n_head_; i++)
      {
        Real xxx = 4.0 * l0_ * std::cos(t);
        Real yyy = 4.0 * l0_ * std::sin(t);
        geom_[1].vertices_.push_back(Vector3<Real>(xxx + xx,
                                                   yyy + yy,
                                                   0));

        u1.push_back(Vector3<Real>(0,0,0));
        f1.push_back(Vector3<Real>(0,0,0));

        t+=(2.0*pi)/Real(n_head_);
      }

      offset_ = springs_.size();

      for(int i(1); i <= n_head_; i++)
      {
        Real lZero = (geom_[1].vertices_[i-1] - geom_[1].vertices_[i%25]).mag();

        geom_[1].faces_.push_back(std::pair<int,int>(i-1,i%25));

        geom_[1].segments_.push_back(Segment3<Real>(  geom_[1].vertices_[i-1], 
                                                      geom_[1].vertices_[i%25]
                                                  ));

        std::pair<int, int> index0(i-1,1);
        std::pair<int, int> index1(i%25,1);
        springs_.push_back(BasicSpringConstraint<Real>(ks_, kd_, lZero, index0, index1,
                                                       &geom_[1].vertices_[i-1],
                                                       &geom_[1].vertices_[i%25],
                                                       &u1[i-1],
                                                       &u1[i%25]
                                                       ));

      }

      for(int k(0); k < n_head_; k++)
      {

        std::pair<int, int> index0(k,1);
        std::pair<int, int> index1((k+2)%25,1);

        Real lZero = (geom_[1].vertices_[k] - geom_[1].vertices_[(k+2)%25]).mag();

        bendSprings_.push_back(
            BasicSpringConstraint<Real>(0.85, -0.25, 
                                        2.0*l0_, index0, index1,
                                        &geom_[1].vertices_[k],
                                        &geom_[1].vertices_[(k+2)%25],
                                        &u1[k],
                                        &u1[(k+2)%25]
                                       ));

      }

      for(int i(0); i < n_head_; i++)
      {

        std::pair<int, int> index0(0,0);
        std::pair<int, int> index1(i,1);
        springs_.push_back(BasicSpringConstraint<Real>(ks_, kd_, 4.0 * l0_, index0, index1,
                                                       &geom_[0].vertices_[0],
                                                       &geom_[1].vertices_[i],
                                                       &u0[0],
                                                       &u1[i]
                                                       ));

      }

    }; 

    void integrate()
    {

      std::vector<Vector3<Real>> &u0 = u_[0]; 
      std::vector<Vector3<Real>> &f0 = force_[0]; 

      std::vector<Vector3<Real>> &u1 = u_[1]; 
      std::vector<Vector3<Real>> &f1 = force_[1]; 

      // Integrate the mid-piece vertices
      for(int i(0); i < n_mid_; ++i)
      {
        Vec3 &vel = u0[i];

        Vec3 &force = f0[i];

        force += -5.0 * 0.0125 * vel;

        Real m = 10.0;
        if(i == 0)m=15000.0;
        else
          m=10.0;

        m = 10.0;

        Vec3 &pos = geom_[0].vertices_[i];
      
        vel = vel + dt_ * force * (1.0/m);

        pos = pos + dt_ * vel;

        force = Vec3(0,0,0);
        if(i==0)
          vel = 0.98 * vel;
      }

      // Integrate the head piece vertices
      for(int i(0); i < geom_[1].vertices_.size(); ++i)
      {
        Vec3 &vel = u1[i];

        Vec3 &force = f1[i];

        force += -5.0 * 0.0125 * vel;

        Real m = 10.0;

        m = 15000.0;

        Vec3 &pos = geom_[1].vertices_[i];
      
        vel = vel + dt_ * force * (1.0/m);

        pos = pos + dt_ * vel;

        force = Vec3(0,0,0);
      }
      
    }; 

  };
  
  template<>
  class SoftBody<Real, ParamLine<Real> > : public BasicSoftBody<Real, ParamLine<Real>>
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

    int n_mid_;

    int n_head_;

    int offset_;

    Vec3 com_;

    Transformationr transform_;

    std::vector< SimpleSpringConstraint<Real> > springs_;

    SoftBody () : A_(3.2), N_(100),  f_(1.0/120.0), a0_(0.5), l0_(1.0*a0_)
    {

    };

    virtual ~SoftBody (){};

    void calcCOM()
    {
      com_ = Vec3(0,0,0);
      for (int i(0); i < geom_.vertices_.size(); ++i)
      {
        com_ += geom_.vertices_[i]; 
      }
      com_ *= (1.0/geom_.vertices_.size());
    }

    void setOrigin(const Vec3 &origin)
    {
      transform_.setOrigin(origin);
    }

    bool isInBody(const Vec3 &vQuery) const
    {
      // transform point to softbody coordinate system 
      Vec3 q = vQuery - transform_.getOrigin(); 
      for (int i(0); i < geom_.vertices_.size(); ++i)
      {
        if((geom_.vertices_[i] - q).mag() < 0.5)
          return true;
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

      // evaluate force for the first 15 segments

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

      const double pi = 3.1415926535897;

      Real q = 4.0 * pi/(l0_ * N_); 

      Real xx = Real(0) * l0_;
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

        geom_.segments_.push_back(Segment3<Real>(  geom_.vertices_[k-1], 
                                                   geom_.vertices_[k]
                                                  ));

        springs_.push_back(SimpleSpringConstraint<Real>(ks_, kd_, l0_, k-1, k,
                                                       &geom_.vertices_[k-1],
                                                       &geom_.vertices_[k]));


      }

    }; 

    void integrate()
    {
      
    }; 

  };
  
} /* i3d */ 

#endif /* end of include guard: SOFTBODY_HPP_NRX1JCK5 */
