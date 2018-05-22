#ifndef SPRINGCONSTRAINT_HPP_3JBQA0L1
#define SPRINGCONSTRAINT_HPP_3JBQA0L1

#include <OgreVector3.h>

using namespace Ogre;

  template <typename T>
  class SimpleSpringConstraint
  {
  public:

    T kd_;
    T ks_;
    T l0_;
    T l_;
    T dt_;

    Vector3 L_;

    Vector3 *x0_;
    Vector3 *x1_;

    Vector3 *v0_;
    Vector3 *v1_;

    int i0_,i1_;

    SimpleSpringConstraint() : ks_(T(0.5)), kd_(T(-0.25)), l0_(T(1)), l_(T(0)), dt_(T(0))
    {

    }

    SimpleSpringConstraint(T _ks, T _kd) : ks_(_ks), kd_(_kd), l0_(T(1)), l_(T(0)), dt_(T(0))
    {

    }

    SimpleSpringConstraint(T _ks, T _kd, T _l0, T _l,   
                          Vector3 *x0, Vector3 *x1,
                          Vector3 *v0, Vector3 *v1)
      : ks_(_ks), kd_(_kd), l0_(_l0), l_(_l)
    {

      x0_ = x0;
      x1_ = x1;

      v0_ = v0;
      v1_ = v1;

    }

    SimpleSpringConstraint(T _ks, T _kd, T _l0, int i0, int i1,    
                          Vector3 *x0, Vector3 *x1,
                          Vector3 *v0, Vector3 *v1)
      : ks_(_ks), kd_(_kd), l0_(_l0), l_(0), i0_(i0), i1_(i1)
    {

      x0_ = x0;
      x1_ = x1;

      v0_ = v0;
      v1_ = v1;

    }

    SimpleSpringConstraint(T _ks, T _kd, T _l0, int _i0, int _i1, 
                          Vector3 *x0, Vector3 *x1)
      : ks_(_ks), kd_(_kd), l0_(_l0), i0_(_i0), i1_(_i1) 
    {

      x0_ = x0;
      x1_ = x1;

    }

    virtual ~SimpleSpringConstraint ()
    {

    };

    virtual Vector3 evalForce()
    {
      Vector3 &x0 = *x0_;
      Vector3 &x1 = *x1_;

      Vector3 &v0 = *v0_;
      Vector3 &v1 = *v1_;

      L_ = (x0 - x1);
      l_ = L_.length();
      if(l_ == 0.0f)
        return Vector3(0,0,0);

      // what if vertices coincide?
      Vector3 LN = (1.0 / l_) * L_;
      T tl          = -ks_ * (l_ - l0_);
      T tr          = kd_ * ( (v0 - v1).dotProduct(L_) )/l_;
      Vector3 f  = (tl + tr) * LN; 

      return f;
    }
  
  private:
    /* data */
  };
  

#endif /* end of include guard: SPRINGCONSTRAINT_HPP_3JBQA0L1 */
