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

    int i0_,i1_;

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

    BasicSpringConstraint(T _ks, T _kd, T _l0, int _i0, int _i1, 
                          Vector3<T> *x0, Vector3<T> *x1)
      : ks_(_ks), kd_(_kd), l0_(_l0), i0_(_i0), i1_(_i1) 
    {

      x0_ = x0;
      x1_ = x1;

    }

    virtual ~BasicSpringConstraint ()
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
