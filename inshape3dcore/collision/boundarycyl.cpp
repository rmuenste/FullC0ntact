#include "boundarycyl.h"

namespace i3d {

template <class T>
CBoundaryCyl<T>::CBoundaryCyl(void)
{
	m_vNormals[0]=CVector3<T>(1,0,0);
	m_vNormals[1]=CVector3<T>(-1,0,0);
	m_vNormals[2]=CVector3<T>(0,1,0);
	m_vNormals[3]=CVector3<T>(0,-1,0);
	m_vNormals[4]=CVector3<T>(0,0,1);
	m_vNormals[5]=CVector3<T>(0,0,-1);
  m_iType = CBoundaryBox<T>::CYLBDRY;
}

template <class T>
CBoundaryCyl<T>::CBoundaryCyl(const CVector3<T> &vOrigin, const T extends[3])
{
	rBox.Init(vOrigin,extends);
	
	m_vNormals[0]=CVector3<T>(1,0,0);
	m_vNormals[1]=CVector3<T>(-1,0,0);
	m_vNormals[2]=CVector3<T>(0,1,0);
	m_vNormals[3]=CVector3<T>(0,-1,0);
	m_vNormals[4]=CVector3<T>(0,0,1);
	m_vNormals[5]=CVector3<T>(0,0,-1);
	
	CalcValues();
	
	rBox.Output();
  m_iType = CBoundaryBox<T>::CYLBDRY;
}

template <class T>
CBoundaryCyl<T>::~CBoundaryCyl(void)
{

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CBoundaryCyl<float>;

template class CBoundaryCyl<double>;
//----------------------------------------------------------------------------

}