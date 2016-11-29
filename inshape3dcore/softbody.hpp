#ifndef SOFTBODY_HPP_NRX1JCK5
#define SOFTBODY_HPP_NRX1JCK5

#include <vector3.h>
#include <segment3.h>
#include <paramline.h>
#include <springconstraint.hpp>
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

    std::vector< BasicSpringConstraint<Real> > springs_;

    SoftBody () : A_(3.2), N_(100),  f_(1.0/120.0), a0_(0.5), l0_(1.0*a0_)
    {

    };

    virtual ~SoftBody (){};

    void internalForce(Real t)
    {

      const double pi = 3.1415926535897;

      Real q = 4.0 * pi/(l0_ * N_); 

      int j = 0;

      for(int i(15); i < N_; ++i)
      {

        Real x = Real(j) * a0_;

        Real xl = -2.0 * pi * f_ * t + q * x;

        Real y = A_ * std::sin(xl);

        geom_.vertices_[i]=Vec3(x,y,0);

        j++;

      }

      // evaluate force for the first 15 segments

    }; 

    void applyForce(Real dt)
    {

    }; 

    void init()
    {


      a0_ = 0.5;

      const double pi = 3.1415926535897;

      Real q = 4.0 * pi/(l0_ * N_); 

      Real xx = Real(-15) * l0_;
      Real yy = A_ * std::sin(-2.0 * pi * f_ + q * xx);

      geom_.vertices_.push_back(Vector3<Real>(xx,
                                              yy,
                                              0));

      int j = -14; 

      for(int k=1; k < N_; ++k)
      {

        Real x = Real(j) * l0_;

        Real y = A_ * std::sin(-2.0 * pi * f_ + q * x);

        geom_.vertices_.push_back(Vector3<Real>(x,y,0));

        geom_.faces_.push_back(std::pair<int,int>(k-1,k));

        geom_.segments_.push_back(Segment3<Real>(  geom_.vertices_[k-1], 
                                                   geom_.vertices_[k]
                                                  ));

        springs_.push_back(BasicSpringConstraint<Real>(ks_, kd_, l0_, k-1, k,
                                                       &geom_.vertices_[k-1],
                                                       &geom_.vertices_[k]));

        j++;

      }

    }; 

    void integrate()
    {
      
    }; 

  };
  
} /* i3d */ 

#endif /* end of include guard: SOFTBODY_HPP_NRX1JCK5 */
