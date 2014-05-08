#include "transform.h"

namespace i3d {
  
template<>
OBB3<Real> CPredictionTransform<Real, OBB3<Real> >::PredictLinearMotion(const OBB3<Real> &box, const Vector3<Real> &vel)
{
  Vector3<Real> vCenter = box.center_ + vel;

  OBB3<Real> newBox(vCenter,box.uvw_,box.extents_);

  return newBox;
}

template<>
OBB3<Real> CPredictionTransform<Real, OBB3<Real> >::PredictMotion(const OBB3<Real> &box, const Vector3<Real> &vel, const Transformation<Real> &transform, const Vector3<Real> &angvel, Real deltaT)
{

  Vector3<Real> vUVW[3];
  Vector3<Real> vVec;

  CMatrix3x3<Real> matAngUpdate = CMatrix3x3<Real>::GetSkewMatrix(angvel);
  const CMatrix3x3<Real> &basis   = transform.getMatrix();
  Vector3<Real> vCenter   = transform.getOrigin() + deltaT * vel;
  CMatrix3x3<Real> mrotMat = basis + (matAngUpdate * basis) * deltaT;

  //transform
  for(int i=0;i<3;i++)
  {
   vVec = box.uvw_[i];
   vVec = mrotMat*vVec;
   vUVW[i] = vVec;
  }

  OBB3<Real> newBox(vCenter,vUVW,box.extents_);

  return newBox;

}


}
