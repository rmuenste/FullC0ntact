#include "distancepointseg.h"

namespace i3d {

template <typename T>
CDistancePointSeg<T>::CDistancePointSeg(void)
{
}

template <typename T>
CDistancePointSeg<T>::CDistancePointSeg(const CVector3<T>& point, const CSegment3<T>& segment)
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
	CVector3<T> diff = m_vPoint - m_Seg.m_vCenter;
	m_ParamSegment = m_Seg.m_vDir * diff;

  //if the projection of the point is
  //on the positive side of v0
	if (-m_Seg.m_Ext < m_ParamSegment)
	{
      //if it is on the segment
			if (m_ParamSegment < m_Seg.m_Ext)
			{
					m_vClosestPoint1 = m_Seg.m_vCenter +
					m_ParamSegment*m_Seg.m_vDir;
			}
			else
			{
          //the projection is on the "right side" of v1 take v1 as closest point
					m_vClosestPoint1 = m_Seg.m_vP1;
					m_ParamSegment = m_Seg.m_Ext;
			}
	}
	else
	{
      //v0 is the closest point
			m_vClosestPoint1 = m_Seg.m_vP0;
			m_ParamSegment = -m_Seg.m_Ext;
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