#include "boundarycyl.h"

namespace i3d {

template <class T>
BoundaryCyl<T>::BoundaryCyl(void)
{
	BoundaryBox<T>::normals_[0]=CVector3<T>(1,0,0);
	BoundaryBox<T>::normals_[1]=CVector3<T>(-1,0,0);
	BoundaryBox<T>::normals_[2]=CVector3<T>(0,1,0);
	BoundaryBox<T>::normals_[3]=CVector3<T>(0,-1,0);
	BoundaryBox<T>::normals_[4]=CVector3<T>(0,0,1);
	BoundaryBox<T>::normals_[5]=CVector3<T>(0,0,-1);
  BoundaryBox<T>::type_ = BoundaryBox<T>::CYLBDRY;
}

template <class T>
BoundaryCyl<T>::BoundaryCyl(const CVector3<T> &vOrigin, const T extends[3])
{
	BoundaryBox<T>::boundingBox_.init(vOrigin,extends);
	
	BoundaryBox<T>::normals_[0]=CVector3<T>(1,0,0);
	BoundaryBox<T>::normals_[1]=CVector3<T>(-1,0,0);
	BoundaryBox<T>::normals_[2]=CVector3<T>(0,1,0);
	BoundaryBox<T>::normals_[3]=CVector3<T>(0,-1,0);
	BoundaryBox<T>::normals_[4]=CVector3<T>(0,0,1);
	BoundaryBox<T>::normals_[5]=CVector3<T>(0,0,-1);
	
	BoundaryBox<T>::calcValues();
	
	BoundaryBox<T>::boundingBox_.Output();
  BoundaryBox<T>::type_ = BoundaryBox<T>::CYLBDRY;
}

template <class T>
BoundaryCyl<T>::~BoundaryCyl(void)
{

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class BoundaryCyl<float>;

template class BoundaryCyl<double>;
//----------------------------------------------------------------------------

}