#include "rectangle3.h"

namespace i3d {

template <typename T>
CRectangle3<T>::CRectangle3(void)
{
}

template <typename T>
CRectangle3<T>::CRectangle3(const CRectangle3<T> &copy)
{
	this->m_vCenter = copy.m_vCenter;
	this->m_vUV[0] = copy.m_vUV[0];
	this->m_vUV[1] = copy.m_vUV[1];
	this->m_Extents[0]=copy.m_Extents[0];
	this->m_Extents[1]=copy.m_Extents[1];
}

template <typename T>
CRectangle3<T>::~CRectangle3(void)
{
}

template <typename T>
CRectangle3<T>::CRectangle3(const CVector3<T> &vCenter, const CVector3<T> &vU, const CVector3<T> &vV, T Extent0, T Extent1)
{
	this->m_vCenter = vCenter;
	this->m_vUV[0] = vU;
	this->m_vUV[1] = vV;
	this->m_Extents[0]=Extent0;
	this->m_Extents[1]=Extent1;
}

template <typename T>
CRectangle3<T>::CRectangle3(const CVector3<T> &vCenter, const CVector3<T> vUV[], const T Extents[])
{
	this->m_vCenter = vCenter;
	this->m_vUV[0] = vUV[0];
	this->m_vUV[1] = vUV[1];
	this->m_Extents[0]=Extents[0];
	this->m_Extents[1]=Extents[1];
}

template <typename T>
void CRectangle3<T>::ComputeVertices(CVector3<T> vVertices[]) const
{
	CVector3<T> vEx0 = m_vUV[0]*m_Extents[0];
	CVector3<T> vEx1 = m_vUV[1]*m_Extents[1];

	vVertices[0] = m_vCenter - vEx0 - vEx1;
	vVertices[1] = m_vCenter + vEx0 - vEx1;
	vVertices[2] = m_vCenter + vEx0 + vEx1;
	vVertices[3] = m_vCenter - vEx0 + vEx1;
}

//template <typename T>
//CVector3<T> CRectangle3<T>::GetMMCorner() const
//{
//
//}
//
//template <typename T>
//CVector3<T> CRectangle3<T>::GetMPCorner() const
//{
//
//}
//
//template <typename T>
//CVector3<T> CRectangle3<T>::GetPMCorner() const
//{
//
//}
//
//template <typename T>
//CVector3<T> CRectangle3<T>::GetPPCorner() const
//{
//
//}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CRectangle3<float>;

template class CRectangle3<double>;
//----------------------------------------------------------------------------

}