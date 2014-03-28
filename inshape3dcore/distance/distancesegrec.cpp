#include <distancelinerec.h>
#include <distancepointrec.h>
#include "distancesegrec.h"

namespace i3d {

template <typename T>
CDistanceSegRec<T>::CDistanceSegRec(void)
{
}

template <typename T>
CDistanceSegRec<T>::CDistanceSegRec(const Segment3<T>& seg, const Rectangle3<T>& rec)
{
  this->m_Seg = seg;
  this->m_Rec  = rec;
}

template <typename T>
T CDistanceSegRec<T>::ComputeDistanceSqr()
{
  CDistanceLineRec<T> queryLR(Line3<T>(m_Seg.center_,
		  m_Seg.dir_), m_Rec);
  T sqrDist = queryLR.ComputeDistanceSqr();
  m_ParamSegment = queryLR.m_ParamLine;

  if (m_ParamSegment >= -m_Seg.ext_)
  {
		  if (m_ParamSegment <= m_Seg.ext_)
		  {
				  m_vClosestPoint0 = queryLR.m_vClosestPoint0;
				  m_vClosestPoint1 = queryLR.m_vClosestPoint1;
				  m_ParamRectangle[0] = queryLR.m_ParamRectangle[0];
				  m_ParamRectangle[1] = queryLR.m_ParamRectangle[1];
		  }
		  else
		  {
				  m_vClosestPoint0 = m_Seg.p1_;
				  CDistancePointRec<T> queryPR(m_vClosestPoint0,m_Rec);
				  sqrDist = queryPR.ComputeDistanceSqr();
				  m_vClosestPoint1 = queryPR.m_vClosestPoint1;
				  m_ParamSegment = m_Seg.ext_;
				  m_ParamRectangle[0] = queryPR.m_ParamRectangle[0];
				  m_ParamRectangle[1] = queryPR.m_ParamRectangle[1];
		  }
  }
  else
  {
		  m_vClosestPoint0 = m_Seg.p0_;
		  CDistancePointRec<T> queryPR(m_vClosestPoint0, m_Rec);
		  sqrDist = queryPR.ComputeDistanceSqr();
		  m_vClosestPoint1 = queryPR.m_vClosestPoint1;
		  m_ParamSegment = -m_Seg.ext_;
		  m_ParamRectangle[0] = queryPR.m_ParamRectangle[0];
		  m_ParamRectangle[1] = queryPR.m_ParamRectangle[1];
  }

  return sqrDist;
}

template <typename T>
T CDistanceSegRec<T>::ComputeDistance()
{
	return sqrt(ComputeDistance());
}

template <typename T>
CDistanceSegRec<T>::~CDistanceSegRec(void)
{

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceSegRec<float>;
template class CDistanceSegRec<double>;

}