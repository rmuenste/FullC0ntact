#include "distancepointrec.h"

namespace i3d {

template <typename T>
CDistancePointRec<T>::CDistancePointRec(void)
{
}

template <typename T>
CDistancePointRec<T>::CDistancePointRec(const Vector3<T>& point, const Rectangle3<T>& rectangle)
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
	Vector3<T> diff = m_Rec.center_ - m_vPoint;
	T b0 = diff * (m_Rec.uv_[0]);
	T b1 = diff * (m_Rec.uv_[1]);
	T s0 = -b0, s1 = -b1;
	T sqrDistance = diff * diff;

	if (s0 < -m_Rec.extents_[0])
	{
		s0 = -m_Rec.extents_[0];
	}
	else if (s0 > m_Rec.extents_[0])
	{
		s0 = m_Rec.extents_[0];
	}
	sqrDistance += s0*(s0 + ((T)2)*b0);

	if (s1 < -m_Rec.extents_[1])
	{
		s1 = -m_Rec.extents_[1];
	}
	else if (s1 > m_Rec.extents_[1])
	{
		s1 = m_Rec.extents_[1];
	}
	sqrDistance += s1*(s1 + ((T)2)*b1);

	// Account for numerical round-off error.
	if (sqrDistance < (T)0)
	{
		sqrDistance = (T)0;
	}

	m_vClosestPoint0 = m_vPoint;
	m_vClosestPoint1 = m_Rec.center_ + m_Rec.uv_[0] * s0 + m_Rec.uv_[1] * s1;
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

template <typename T>
DistancePointPlane<T>::DistancePointPlane(void)
{
}

template <typename T>
DistancePointPlane<T>::DistancePointPlane(const Vector3<T>& point, const Plane<T>& plane)
{
	plane_ = plane;
	m_vPoint = point;
}

template <typename T>
DistancePointPlane<T>::~DistancePointPlane(void)
{
}

template <typename T>
T DistancePointPlane<T>::ComputeDistanceSqr()
{
  return T(0);
}

template <typename T>
T DistancePointPlane<T>::ComputeDistance()
{
  Vector3<T> diff = m_vPoint - plane_.m_vOrigin;

  T signedDist = diff * plane_.m_vNormal; 

  m_vClosestPoint0 = m_vPoint;
  m_vClosestPoint1 = m_vPoint - (signedDist * plane_.m_vNormal);

  return std::abs(signedDist);
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class DistancePointPlane<float>;
template class DistancePointPlane<double>;

}