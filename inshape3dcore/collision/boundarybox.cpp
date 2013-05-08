#include "boundarybox.h"

namespace i3d {

template <class T>
CBoundaryBox<T>::CBoundaryBox(void)
{
	m_vNormals[0]=CVector3<T>(1,0,0);
	m_vNormals[1]=CVector3<T>(-1,0,0);
	m_vNormals[2]=CVector3<T>(0,1,0);
	m_vNormals[3]=CVector3<T>(0,-1,0);
	m_vNormals[4]=CVector3<T>(0,0,1);
	m_vNormals[5]=CVector3<T>(0,0,-1);
  m_iType = CBoundaryBox::BOXBDRY;
}

template <class T>
CBoundaryBox<T>::CBoundaryBox(const CVector3<T> &vOrigin, const T extends[3])
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
  m_iType = CBoundaryBox::BOXBDRY;
}

template <class T>
CBoundaryBox<T>::~CBoundaryBox(void)
{

}

template <class T>
void CBoundaryBox<T>::CalcValues()
{
	m_Values[0]=rBox.Xmin();
	m_Values[1]=rBox.Xmax();
	m_Values[2]=rBox.Ymin();
	m_Values[3]=rBox.Ymax();
	m_Values[4]=rBox.Zmin();
	m_Values[5]=rBox.Zmax();

	Real x2 = (m_Values[1] - m_Values[0])/2.0;
	Real y2 = (m_Values[3] - m_Values[2])/2.0;
	Real z2 = (m_Values[5] - m_Values[4])/2.0;

	m_vPoints[0]=CVector3<T>(m_Values[0],y2,z2);
	m_vPoints[1]=CVector3<T>(m_Values[1],y2,z2);
	m_vPoints[2]=CVector3<T>(x2,m_Values[2],z2);
	m_vPoints[3]=CVector3<T>(x2,m_Values[3],z2);
	m_vPoints[4]=CVector3<T>(x2,y2,m_Values[4]);
	m_vPoints[5]=CVector3<T>(x2,y2,m_Values[5]);
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CBoundaryBox<float>;

template class CBoundaryBox<double>;
//----------------------------------------------------------------------------

}