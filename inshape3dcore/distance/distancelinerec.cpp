#include "distancelinerec.h"
#include <distancelineseg.h>

namespace i3d {

template <typename T>
CDistanceLineRec<T>::CDistanceLineRec(void)
{
}

template <typename T>
CDistanceLineRec<T>::CDistanceLineRec(const Line3<T>& line, const Rectangle3<T>& rec)
{
  this->m_Line = line;
  this->m_Rec  = rec;
}

template <typename T>
CDistanceLineRec<T>::~CDistanceLineRec(void)
{
}

template <typename T>
T CDistanceLineRec<T>::ComputeDistance()
{
 return sqrt(ComputeDistanceSqr());
}

template <typename T>
T CDistanceLineRec<T>::ComputeDistanceSqr()
{
  // Test if line intersects rectangle.  If so, the squared distance is
  // zero.
  CVector3<T> N = CVector3<T>::Cross(m_Rec.uv_[0],m_Rec.uv_[1]);
  T NdD = N * m_Line.dir_;
  if (fabs(NdD) > E5)
  {
    // The line and rectangle are not parallel, so the line intersects
    // the plane of the rectangle.
    CVector3<T> diff = m_Line.origin_ - m_Rec.center_;
    CVector3<T> U, V;
    CVector3<T>::GenerateComplementBasis(U, V, m_Line.dir_);
    T UdD0 = U * m_Rec.uv_[0];
    T UdD1 = U * m_Rec.uv_[1];
    T UdPmC = U * diff;
    T VdD0 = V * m_Rec.uv_[0];
    T VdD1 = V * m_Rec.uv_[1];
    T VdPmC = V * diff;
    T invDet = ((T)1)/(UdD0*VdD1 - UdD1*VdD0);

    // Rectangle coordinates for the point of intersection.
    T s0 = (VdD1*UdPmC - UdD1*VdPmC)*invDet;
    T s1 = (UdD0*VdPmC - VdD0*UdPmC)*invDet;

    if (fabs(s0) <= m_Rec.extents_[0] && fabs(s1) <= m_Rec.extents_[1])
    {
      // Line parameter for the point of intersection.
      T DdD0 = m_Line.dir_ * m_Rec.uv_[0];
      T DdD1 = m_Line.dir_ * m_Rec.uv_[1];
      T DdDiff = m_Line.dir_ * diff;
      m_ParamLine = s0*DdD0 + s1*DdD1 - DdDiff;

      // Rectangle coordinates for the point of intersection.
      m_ParamRectangle[0] = s0;
      m_ParamRectangle[1] = s1;

      // The intersection point is inside or on the rectangle.
      m_vClosestPoint0 = m_Line.origin_ + m_ParamLine*m_Line.dir_;

      m_vClosestPoint1 = m_Rec.center_ + s0*m_Rec.uv_[0] + s1*m_Rec.uv_[1];

      return (T)0;
    }
  }

  // Either (1) the line is not parallel to the rectangle and the point of
  // intersection of the line and the plane of the rectangle is outside the
  // rectangle or (2) the line and rectangle are parallel.  Regardless, the
  // closest point on the rectangle is on an edge of the rectangle.  Compare
  // the line to all four edges of the rectangle.
  T sqrDist = std::numeric_limits<T>::max();
  
  CVector3<T> scaledDir[2] =  {m_Rec.extents_[0]*m_Rec.uv_[0], m_Rec.extents_[1]*m_Rec.uv_[1]};
  
  for (int i1 = 0; i1 < 2; ++i1)
  {
    for (int i0 = 0; i0 < 2; ++i0)
    {
      Segment3<T> segment;
      segment.center_ = m_Rec.center_ +
      ((T)(2*i0-1))*scaledDir[i1];
      segment.dir_ = m_Rec.uv_[1-i1];
      segment.ext_ = m_Rec.extents_[1-i1];
      segment.calcVertices();

      CDistanceLineSeg<T> queryLS(m_Line, segment);
      T sqrDistTmp = queryLS.ComputeDistanceSqr();
      if (sqrDistTmp < sqrDist)
      {
        m_vClosestPoint0 = queryLS.m_vClosestPoint0;
        m_vClosestPoint1 = queryLS.m_vClosestPoint1;
        sqrDist = sqrDistTmp;

        m_ParamLine = queryLS.m_ParamLine;
        T ratio = queryLS.m_ParamSegment/segment.ext_;
        m_ParamRectangle[0] = m_Rec.extents_[0]*((1-i1)*(2*i0-1) + i1*ratio);
        m_ParamRectangle[1] = m_Rec.extents_[1]*((1-i0)*(2*i1-1) + i0*ratio);
      }
    }
  }

  return sqrDist;

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceLineRec<float>;

template class CDistanceLineRec<double>;
//----------------------------------------------------------------------------

}