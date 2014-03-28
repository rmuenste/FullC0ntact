#include "boundarybox.h"

namespace i3d {

template <class T>
BoundaryBox<T>::BoundaryBox(void)
{
  normals_[0] = CVector3<T>(1,0,0);
  normals_[1] = CVector3<T>(-1,0,0);
  normals_[2] = CVector3<T>(0,1,0);
  normals_[3] = CVector3<T>(0,-1,0);
  normals_[4] = CVector3<T>(0,0,1);
  normals_[5] = CVector3<T>(0,0,-1);
  type_       = BoundaryBox::BOXBDRY;
}

template <class T>
BoundaryBox<T>::BoundaryBox(const CVector3<T> &vOrigin, const T extends[3])
{
  boundingBox_.init(vOrigin,extends);

  normals_[0] = CVector3<T>(1,0,0);
  normals_[1] = CVector3<T>(-1,0,0);
  normals_[2] = CVector3<T>(0,1,0);
  normals_[3] = CVector3<T>(0,-1,0);
  normals_[4] = CVector3<T>(0,0,1);
  normals_[5] = CVector3<T>(0,0,-1);

  calcValues();

  boundingBox_.Output();
  type_ = BoundaryBox::BOXBDRY;
}

template <class T>
BoundaryBox<T>::~BoundaryBox(void)
{

}

template <class T>
void BoundaryBox<T>::calcValues()
{
  extents_[0]=boundingBox_.xmin();
  extents_[1]=boundingBox_.xmax();
  extents_[2]=boundingBox_.ymin();
  extents_[3]=boundingBox_.ymax();
  extents_[4]=boundingBox_.zmin();
  extents_[5]=boundingBox_.zmax();

  Real x2 = (extents_[1] - extents_[0])/2.0;
  Real y2 = (extents_[3] - extents_[2])/2.0;
  Real z2 = (extents_[5] - extents_[4])/2.0;

  points_[0]=CVector3<T>(extents_[0],y2,z2);
  points_[1]=CVector3<T>(extents_[1],y2,z2);
  points_[2]=CVector3<T>(x2,extents_[2],z2);
  points_[3]=CVector3<T>(x2,extents_[3],z2);
  points_[4]=CVector3<T>(x2,y2,extents_[4]);
  points_[5]=CVector3<T>(x2,y2,extents_[5]);
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class BoundaryBox<float>;

template class BoundaryBox<double>;
//----------------------------------------------------------------------------

}