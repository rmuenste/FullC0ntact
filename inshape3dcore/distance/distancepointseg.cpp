#include "distancepointseg.h"

namespace i3d {

template <typename T>
CDistancePointSeg<T>::CDistancePointSeg(void)
{
}

template <typename T>
CDistancePointSeg<T>::CDistancePointSeg(const Vector3<T>& point, const Segment3<T>& segment)
{
  m_Seg = segment;
  m_vPoint = point;
}

template <typename T>
CDistancePointSeg<T>::~CDistancePointSeg(void)
{
}

template <typename T>
T CDistancePointSeg<T>::ComputeDistanceSqr()
{
  Vector3<T> diff = m_vPoint - m_Seg.center_;
  m_ParamSegment = m_Seg.dir_ * diff;

  //if the projection of the point is
  //on the positive side of v0
  if (-m_Seg.ext_ < m_ParamSegment)
  {
    //if it is on the segment
    if (m_ParamSegment < m_Seg.ext_)
    {
      m_vClosestPoint1 = m_Seg.center_ +
      m_ParamSegment*m_Seg.dir_;
    }
    else
    {
      //the projection is on the "right side" of v1 take v1 as closest point
      m_vClosestPoint1 = m_Seg.p1_;
      m_ParamSegment = m_Seg.ext_;
    }
  }
  else
  {
    //v0 is the closest point
    m_vClosestPoint1 = m_Seg.p0_;
    m_ParamSegment = -m_Seg.ext_;
  }

  m_vClosestPoint0 = m_vPoint;
  diff = m_vClosestPoint1 - m_vClosestPoint0;
  return (diff*diff);
}

template <typename T>
T CDistancePointSeg<T>::ComputeDistance()
{
	return sqrt(ComputeDistanceSqr());
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistancePointSeg<float>;
template class CDistancePointSeg<double>;

}