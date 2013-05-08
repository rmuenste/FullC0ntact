#include "boundarycyl.h"

namespace i3d {

template <class T>
CBoundaryCyl<T>::CBoundaryCyl(void)
{
	CBoundaryBox<T>::m_vNormals[0]=CVector3<T>(1,0,0);
	CBoundaryBox<T>::m_vNormals[1]=CVector3<T>(-1,0,0);
	CBoundaryBox<T>::m_vNormals[2]=CVector3<T>(0,1,0);
	CBoundaryBox<T>::m_vNormals[3]=CVector3<T>(0,-1,0);
	CBoundaryBox<T>::m_vNormals[4]=CVector3<T>(0,0,1);
	CBoundaryBox<T>::m_vNormals[5]=CVector3<T>(0,0,-1);
  CBoundaryBox<T>::m_iType = CBoundaryBox<T>::CYLBDRY;
}

template <class T>
CBoundaryCyl<T>::CBoundaryCyl(const CVector3<T> &vOrigin, const T extends[3])
{
	CBoundaryBox<T>::rBox.Init(vOrigin,extends);
	
	CBoundaryBox<T>::m_vNormals[0]=CVector3<T>(1,0,0);
	CBoundaryBox<T>::m_vNormals[1]=CVector3<T>(-1,0,0);
	CBoundaryBox<T>::m_vNormals[2]=CVector3<T>(0,1,0);
	CBoundaryBox<T>::m_vNormals[3]=CVector3<T>(0,-1,0);
	CBoundaryBox<T>::m_vNormals[4]=CVector3<T>(0,0,1);
	CBoundaryBox<T>::m_vNormals[5]=CVector3<T>(0,0,-1);
	
	CBoundaryBox<T>::CalcValues();
	
	CBoundaryBox<T>::rBox.Output();
  CBoundaryBox<T>::m_iType = CBoundaryBox<T>::CYLBDRY;
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