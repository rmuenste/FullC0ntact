#include "distancepointpline.h"

namespace i3d {

template <typename T>
CDistancePointPline<T>::CDistancePointPline(void)
{
}

template <typename T>
CDistancePointPline<T>::CDistancePointPline(const CVector3<T>& point, CParamLine<T>& line)
{
	m_pLine = &line;
	m_vPoint = point;
}

template <typename T>
CDistancePointPline<T>::~CDistancePointPline(void)
{
}

template <typename T>
T CDistancePointPline<T>::ComputeDistanceSqr()
{
  T res = ComputeDistance();
  return res*res;
}

template <typename T>
T CDistancePointPline<T>::ComputeDistance()
{
  T mindist = std::numeric_limits<T>::max();
  typename std::vector< CSegment3<T> >::iterator iter = m_pLine->m_pSegments.begin();
  for(;iter!=m_pLine->m_pSegments.end();iter++)
  {
    CSegment3<T> &seg = *iter;
    CDistancePointSeg<T> distPointSeg(m_vPoint,seg);
    T dist = distPointSeg.ComputeDistance();
    if(dist < mindist)
    {
      mindist=dist;
      m_vClosestPoint1 = distPointSeg.m_vClosestPoint1;
    }
  }
  return mindist;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistancePointPline<float>;
template class CDistancePointPline<double>;

}