#ifndef SPRINGCONSTRAINT_HPP_3JBQA0L1
#define SPRINGCONSTRAINT_HPP_3JBQA0L1

#include <vector3.h>

namespace i3d
{
  template <typename T>
  class BasicSpringConstraint
  {
  public:

    T kd_;
    T ks_;
    T l0_;
    T l_;
    T dt_;

    Vector3<T> L_;

    Vector3<T> *x0_;
    Vector3<T> *x1_;

    Vector3<T> *v0_;
    Vector3<T> *v1_;

    std::pair<int,int> i0_,i1_;

    BasicSpringConstraint() : ks_(T(0.5)), kd_(T(-0.25)), l0_(T(1)), l_(T(0)), dt_(T(0))
    {

    }

    BasicSpringConstraint(T _ks, T _kd) : ks_(_ks), kd_(_kd), l0_(T(1)), l_(T(0)), dt_(T(0))
    {

    }

    BasicSpringConstraint(T _ks, T _kd, T _l0, T _l,   
                          Vector3<T> *x0, Vector3<T> *x1,
                          Vector3<T> *v0, Vector3<T> *v1)
      : ks_(_ks), kd_(_kd), l0_(_l0), l_(_l)
    {

      x0_ = x0;
      x1_ = x1;

      v0_ = v0;
      v1_ = v1;

    }

    BasicSpringConstraint(T _ks, T _kd, T _l0, std::pair<int,int> _i0, std::pair<int,int> _i1, 
                          Vector3<T> *x0, Vector3<T> *x1)
      : ks_(_ks), kd_(_kd), l0_(_l0), i0_(_i0), i1_(_i1) 
    {

      x0_ = x0;
      x1_ = x1;

    }

    BasicSpringConstraint(T _ks, T _kd, T _l0, std::pair<int,int> _i0, std::pair<int,int> _i1, 
                          Vector3<T> *x0, Vector3<T> *x1,
                          Vector3<T> *v0, Vector3<T> *v1)
      : ks_(_ks), kd_(_kd), l0_(_l0), i0_(_i0), i1_(_i1) 
    {

      x0_ = x0;
      x1_ = x1;
      v0_ = v0;
      v1_ = v1;

    }

    BasicSpringConstraint(const BasicSpringConstraint &copy)
    {

      kd_ = copy.kd_;
      ks_ = copy.ks_;
      l0_ = copy.l0_;
      l_  = copy.l_;
      dt_ = copy.dt_;

      L_  = copy.L_;

      x0_ = copy.x0_;
      x1_ = copy.x1_;

      v0_ = copy.v0_;
      v1_ = copy.v1_;

      i0_ = copy.i0_;
      i1_ = copy.i1_;

    }

    virtual ~BasicSpringConstraint ()
    {

    };


    // Returns the spring force
    virtual Vector3<T> evalForce()
    {
      Vector3<T> &x0 = *x0_;
      Vector3<T> &x1 = *x1_;

      Vector3<T> &v0 = *v0_;
      Vector3<T> &v1 = *v1_;

      L_ = (x0 - x1);
      l_ = L_.mag();
      if(l_ == 0.0f)
        return Vector3<T>(0,0,0);

      // what if vertices coincide?
      Vector3<T> LN = (1.0 / l_) * L_;
      T tl          = -ks_ * (l_ - l0_);
      T tr          = kd_ * ((v0 - v1) * L_)/l_;
      Vector3<T> f  = (tl + tr) * LN; 

      return f;
    }

    // Returns the spring force
    Vector3<T> evalForce(Vector3<T> &x0, Vector3<T> &x1,
                         Vector3<T> &v0, Vector3<T> &v1
                        )
    {

      L_ = (x0 - x1);
      l_ = L_.mag();
      if(l_ == 0.0f)
        return Vector3<T>(0,0,0);

      // what if vertices coincide?
      Vector3<T> LN = (1.0 / l_) * L_;
      T tl          = -ks_ * (l_ - l0_);
      T tr          = kd_ * ((v0 - v1) * L_)/l_;
      Vector3<T> f  = (tl + tr) * LN; 

      return f;
    }
  
  private:
    /* data */
  };

  template <typename T>
  class SimpleSpringConstraint
  {
  public:

    T kd_;
    T ks_;
    T l0_;
    T l_;
    T dt_;

    Vector3<T> L_;

    Vector3<T> *x0_;
    Vector3<T> *x1_;

    Vector3<T> *v0_;
    Vector3<T> *v1_;

    int i0_,i1_;

    SimpleSpringConstraint() : ks_(T(0.5)), kd_(T(-0.25)), l0_(T(1)), l_(T(0)), dt_(T(0))
    {

    }

    SimpleSpringConstraint(T _ks, T _kd) : ks_(_ks), kd_(_kd), l0_(T(1)), l_(T(0)), dt_(T(0))
    {

    }

    SimpleSpringConstraint(T _ks, T _kd, T _l0, T _l,   
                          Vector3<T> *x0, Vector3<T> *x1,
                          Vector3<T> *v0, Vector3<T> *v1)
      : ks_(_ks), kd_(_kd), l0_(_l0), l_(_l)
    {

      x0_ = x0;
      x1_ = x1;

      v0_ = v0;
      v1_ = v1;

    }

    SimpleSpringConstraint(T _ks, T _kd, T _l0, int i0, int i1,    
                          Vector3<T> *x0, Vector3<T> *x1,
                          Vector3<T> *v0, Vector3<T> *v1)
      : ks_(_ks), kd_(_kd), l0_(_l0), l_(0), i0_(i0), i1_(i1)
    {

      x0_ = x0;
      x1_ = x1;

      v0_ = v0;
      v1_ = v1;

    }

    SimpleSpringConstraint(T _ks, T _kd, T _l0, int _i0, int _i1, 
                          Vector3<T> *x0, Vector3<T> *x1)
      : ks_(_ks), kd_(_kd), l0_(_l0), i0_(_i0), i1_(_i1) 
    {

      x0_ = x0;
      x1_ = x1;

    }

    virtual ~SimpleSpringConstraint ()
    {

    };

    virtual Vector3<T> evalForce()
    {
      Vector3<T> &x0 = *x0_;
      Vector3<T> &x1 = *x1_;

      Vector3<T> &v0 = *v0_;
      Vector3<T> &v1 = *v1_;

      L_ = (x0 - x1);
      l_ = L_.mag();
      if(l_ == 0.0f)
        return Vector3<T>(0,0,0);

      // what if vertices coincide?
      Vector3<T> LN = (1.0 / l_) * L_;
      T tl          = -ks_ * (l_ - l0_);
      T tr          = kd_ * ((v0 - v1) * L_)/l_;
      Vector3<T> f  = (tl + tr) * LN; 

      return f;
    }
  
  private:
    /* data */
  };
  
} /* i3d */ 

#endif /* end of include guard: SPRINGCONSTRAINT_HPP_3JBQA0L1 */
