#include "distancelineline.h"

namespace i3d {

template <typename T>
CDistanceLineLine<T>::CDistanceLineLine(void)
{

}

template <typename T>
CDistanceLineLine<T>::~CDistanceLineLine(void)
{

}

template <typename T>
CDistanceLineLine<T>::CDistanceLineLine(const Line3<T> &Line0, const Line3<T> &Line1) : m_Line0(Line0), m_Line1(Line1)
{

}

template <typename T>
T CDistanceLineLine<T>::ComputeDistance()
{
  return sqrt(ComputeDistanceSqr());
}

template <typename T>
T CDistanceLineLine<T>::ComputeDistanceSqr()
{
  Vector3<T> diff = m_Line0.origin_ - m_Line1.origin_;
  T a01 = -m_Line0.dir_ * m_Line1.dir_;
  T b0 = diff * m_Line0.dir_;
  T c = diff * diff;
  T det = fabs((T)1 - a01*a01);
  T b1, s0, s1, sqrDist;

  //if the quadratic is greater than 0
  //the lines are not parallel, because length(cross(dir0,dir1))^2 > 0
  if (det >= E5)
  {
    // Lines are not parallel.
    b1 = -diff * m_Line1.dir_;
    T invDet = ((T)1)/det;
    s0 = (a01*b1 - b0)*invDet;
    s1 = (a01*b0 - b1)*invDet;
    sqrDist = s0*(s0 + a01*s1 + ((T)2)*b0) +
    s1*(a01*s0 + s1 + ((T)2)*b1) + c;
  }
  else
  {
    // Lines are parallel, select any closest pair of points.
    s0 = -b0;
    s1 = (T)0;
    sqrDist = b0*s0 + c;
  }

  m_vClosestPoint0 = m_Line0.origin_ + m_Line0.dir_ * s0;
  m_vClosestPoint1 = m_Line1.origin_ + m_Line1.dir_ * s1;

  m_Line0Param = s0;
  m_Line1Param = s1;

  // Account for numerical round-off errors.
  if (sqrDist < (T)0)
  {
    sqrDist = (T)0;
  }
  
  return sqrDist;

}
//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceLineLine<float>;

template class CDistanceLineLine<double>;
//----------------------------------------------------------------------------

}