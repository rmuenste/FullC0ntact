#include "distanceobb3plane.h"

namespace i3d {

template <typename T>
CDistanceOBB3Plane<T>::~CDistanceOBB3Plane(void)
{
}

template <typename T>
T CDistanceOBB3Plane<T>::ComputeDistanceSqr()
{
	return T(1.0);
}

template <typename T>
T CDistanceOBB3Plane<T>::ComputeDistance()
{

	Vector3<T> m_pVertices[8];
	m_pBox->computeVertices(m_pVertices);

	T mindist=std::numeric_limits<T>::max();
	int imin=-1;

	for(int i=0;i<7;i++)
	{
		Vector3<T> vP = m_pVertices[i] - m_vPoint;

		T dist = m_vNormal * vP;
		if(dist < mindist)
		{
			mindist = dist;
			imin = i;
		}
	}

	//compute the closest point on the plane
	if(mindist >= 0)
		m_vClosestPoint0 = m_pVertices[imin] - mindist * m_vNormal;
	else
		m_vClosestPoint0 = m_pVertices[imin] + mindist * m_vNormal;

	//the closest point on the obb
	m_vClosestPoint1 = m_pVertices[imin];

	return mindist;

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceOBB3Plane<float>;
template class CDistanceOBB3Plane<double>;

}
