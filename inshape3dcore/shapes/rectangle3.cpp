#include "rectangle3.h"

namespace i3d {


template <typename T>
Rectangle3<T>::Rectangle3(void)
{
}

template <typename T>
Rectangle3<T>::Rectangle3(const Rectangle3<T> &copy)
{
  this->center_ = copy.center_;
  this->uv_[0] = copy.uv_[0];
  this->uv_[1] = copy.uv_[1];
  this->extents_[0]=copy.extents_[0];
  this->extents_[1]=copy.extents_[1];
}

template <typename T>
Rectangle3<T>::~Rectangle3(void)
{
}

template <typename T>
Rectangle3<T>::Rectangle3(const CVector3<T> &vCenter, const CVector3<T> &vU, const CVector3<T> &vV, T Extent0, T Extent1)
{
  this->center_ = vCenter;
  this->uv_[0] = vU;
  this->uv_[1] = vV;
  this->extents_[0]=Extent0;
  this->extents_[1]=Extent1;
}

template <typename T>
Rectangle3<T>::Rectangle3(const CVector3<T> &vCenter, const CVector3<T> vUV[], const T Extents[])
{
  this->center_ = vCenter;
  this->uv_[0] = vUV[0];
  this->uv_[1] = vUV[1];
  this->extents_[0]=Extents[0];
  this->extents_[1]=Extents[1];
}

template <typename T>
void Rectangle3<T>::computeVertices(CVector3<T> vVertices[]) const
{
  CVector3<T> vEx0 = uv_[0]*extents_[0];
  CVector3<T> vEx1 = uv_[1]*extents_[1];

  vVertices[0] = center_ - vEx0 - vEx1;
  vVertices[1] = center_ + vEx0 - vEx1;
  vVertices[2] = center_ + vEx0 + vEx1;
  vVertices[3] = center_ - vEx0 + vEx1;
}

  
  
//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class Rectangle3<float>;

template class Rectangle3<double>;
//----------------------------------------------------------------------------

}