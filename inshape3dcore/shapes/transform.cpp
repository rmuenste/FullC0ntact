#include "transform.h"

namespace i3d {
  
template<>
OBB3<Real> CPredictionTransform<Real, OBB3<Real> >::PredictLinearMotion(const OBB3<Real> &box, const CVector3<Real> &vel)
{
  CVector3<Real> vCenter = box.center_ + vel;

  OBB3<Real> newBox(vCenter,box.uvw_,box.extents_);

  return newBox;
}

template<>
OBB3<Real> CPredictionTransform<Real, OBB3<Real> >::PredictMotion(const OBB3<Real> &box, const CVector3<Real> &vel, const Transformation<Real> &transform, const CVector3<Real> &angvel, Real deltaT)
{

  CVector3<Real> vUVW[3];
  CVector3<Real> vVec;

  CMatrix3x3<Real> matAngUpdate = CMatrix3x3<Real>::GetSkewMatrix(angvel);
  const CMatrix3x3<Real> &basis   = transform.getMatrix();
  CVector3<Real> vCenter   = transform.getOrigin() + deltaT * vel;
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
