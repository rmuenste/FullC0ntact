#include "transform.h"

namespace i3d {
  
template<>
COBB3<Real> CPredictionTransform<Real, COBB3<Real> >::PredictLinearMotion(const COBB3<Real> &box, const CVector3<Real> &vel)
{
  CVector3<Real> vCenter = box.m_vCenter + vel;

  COBB3<Real> newBox(vCenter,box.m_vUVW,box.m_Extents);

  return newBox;
}

template<>
COBB3<Real> CPredictionTransform<Real, COBB3<Real> >::PredictMotion(const COBB3<Real> &box, const CVector3<Real> &vel, const CTransform<Real> &transform, const CVector3<Real> &angvel, Real deltaT)
{

  CVector3<Real> vUVW[3];
  CVector3<Real> vVec;

  CMatrix3x3<Real> matAngUpdate = CMatrix3x3<Real>::GetSkewMatrix(angvel);
  const CMatrix3x3<Real> &basis   = transform.GetTransformation();
  CVector3<Real> vCenter   = transform.GetOrigin() + deltaT * vel;
  CMatrix3x3<Real> mrotMat = basis + (matAngUpdate * basis) * deltaT;

  //transform
  for(int i=0;i<3;i++)
  {
   vVec = box.m_vUVW[i];
   vVec = mrotMat*vVec;
   vUVW[i] = vVec;
  }

  COBB3<Real> newBox(vCenter,vUVW,box.m_Extents);

  return newBox;

}


}
