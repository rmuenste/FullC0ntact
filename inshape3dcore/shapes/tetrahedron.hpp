#ifndef TETRAHEDRON_HPP_VZBTQI7B
#define TETRAHEDRON_HPP_VZBTQI7B
#include <vector3.h>

namespace i3d {

template<typename T>
class Tetrahedron 
{

  public:

  Vector3<T> va_;
  Vector3<T> vb_;
  Vector3<T> vc_;
  Vector3<T> vd_;

  Vector3<T> na_;
  Vector3<T> nb_;
  Vector3<T> nc_;
  Vector3<T> nd_;


  
  Tetrahedron()
  {

  };

  Tetrahedron(const Vector3<T> &a, const Vector3<T> &b, const Vector3<T> &c, const Vector3<T> &d) : va_(a), vb_(b), vc_(c), vd_(d)
  {

    na_ = Vector3<T>::Cross( (vb_-va_), (vc_-va_));
    nb_ = Vector3<T>::Cross( (vc_-vb_), (vd_-vb_));
    nc_ = Vector3<T>::Cross( (vd_-vc_), (va_-vc_));
    nd_ = Vector3<T>::Cross( (va_-vd_), (vb_-vd_));
    na_.normalize();
    nb_.normalize();
    nc_.normalize();
    nd_.normalize();
//    std::cout << "face a: " << na_ << std::endl;
//    std::cout << "face b: " << nb_ << std::endl;
//    std::cout << "face c: " << nc_ << std::endl;
//    std::cout << "face c: " << nd_ << std::endl;
  };

  ~Tetrahedron()
  {

  };

  T signedDistance(const Vector3<T> &p, unsigned which)
  {

    if(which == 0)
    {
      return na_ * (p - va_);
    }
    else if(which == 1)
    {
      return nb_ * (p - vb_);
    }
    else if(which == 2)
    {
      return nc_ * (p - vc_);
    }
    else if(which == 3)
    {
      return nd_ * (p - vd_);
    }
    else
      return -1.0;

  }

  void barycentricCoords(const Vector3<T> &p, T &ea, T &eb, T &ec, T &ed)
  {

    ea = (signedDistance(p,1)/signedDistance(va_,1));

    eb = (signedDistance(p,2)/signedDistance(vb_,2));

    ec = (signedDistance(p,3)/signedDistance(vc_,3));

    ed = (signedDistance(p,0)/signedDistance(vd_,0));

  }


};


};


#endif /* end of include guard: TETRAHEDRON_HPP_VZBTQI7B */
