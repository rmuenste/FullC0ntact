#include "distancepointrec.h"

namespace i3d {

template <typename T>
CDistancePointRec<T>::CDistancePointRec(void)
{
}

template <typename T>
CDistancePointRec<T>::CDistancePointRec(const CVector3<T>& point, const CRectangle3<T>& rectangle)
{
	m_Rec = rectangle;
	m_vPoint = point;
}



template <typename T>
CDistancePointRec<T>::~CDistancePointRec(void)
{
}

template <typename T>
T CDistancePointRec<T>::ComputeDistanceSqr()
{
	CVector3<T> diff = m_Rec.m_vCenter - m_vPoint;
	T b0 = diff * (m_Rec.m_vUV[0]);
	T b1 = diff * (m_Rec.m_vUV[1]);
	T s0 = -b0, s1 = -b1;
	T sqrDistance = diff * diff;

	if (s0 < -m_Rec.m_Extents[0])
	{
		s0 = -m_Rec.m_Extents[0];
	}
	else if (s0 > m_Rec.m_Extents[0])
	{
		s0 = m_Rec.m_Extents[0];
	}
	sqrDistance += s0*(s0 + ((T)2)*b0);

	if (s1 < -m_Rec.m_Extents[1])
	{
		s1 = -m_Rec.m_Extents[1];
	}
	else if (s1 > m_Rec.m_Extents[1])
	{
		s1 = m_Rec.m_Extents[1];
	}
	sqrDistance += s1*(s1 + ((T)2)*b1);

	// Account for numerical round-off error.
	if (sqrDistance < (T)0)
	{
		sqrDistance = (T)0;
	}

	m_vClosestPoint0 = m_vPoint;
	m_vClosestPoint1 = m_Rec.m_vCenter + m_Rec.m_vUV[0] * s0 + m_Rec.m_vUV[1] * s1;
	m_ParamRectangle[0] = s0;
	m_ParamRectangle[1] = s1;

	return sqrDistance;
}

template <typename T>
T CDistancePointRec<T>::ComputeDistance()
{
	return sqrt(ComputeDistanceSqr());
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistancePointRec<float>;
template class CDistancePointRec<double>;

}