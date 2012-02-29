#include <distancelinerec.h>
#include <distancepointrec.h>
#include "distancesegrec.h"

namespace i3d {

template <typename T>
CDistanceSegRec<T>::CDistanceSegRec(void)
{
}

template <typename T>
CDistanceSegRec<T>::CDistanceSegRec(const CSegment3<T>& seg, const CRectangle3<T>& rec)
{
	this->m_Seg = seg;
	this->m_Rec  = rec;
}

template <typename T>
T CDistanceSegRec<T>::ComputeDistanceSqr()
{
		CDistanceLineRec<T> queryLR(CLine3<T>(m_Seg.m_vCenter,
				m_Seg.m_vDir), m_Rec);
		T sqrDist = queryLR.ComputeDistanceSqr();
		m_ParamSegment = queryLR.m_ParamLine;

		if (m_ParamSegment >= -m_Seg.m_Ext)
		{
				if (m_ParamSegment <= m_Seg.m_Ext)
				{
						m_vClosestPoint0 = queryLR.m_vClosestPoint0;
						m_vClosestPoint1 = queryLR.m_vClosestPoint1;
						m_ParamRectangle[0] = queryLR.m_ParamRectangle[0];
						m_ParamRectangle[1] = queryLR.m_ParamRectangle[1];
				}
				else
				{
						m_vClosestPoint0 = m_Seg.m_vP1;
						CDistancePointRec<T> queryPR(m_vClosestPoint0,m_Rec);
						sqrDist = queryPR.ComputeDistanceSqr();
						m_vClosestPoint1 = queryPR.m_vClosestPoint1;
						m_ParamSegment = m_Seg.m_Ext;
						m_ParamRectangle[0] = queryPR.m_ParamRectangle[0];
						m_ParamRectangle[1] = queryPR.m_ParamRectangle[1];
				}
		}
		else
		{
				m_vClosestPoint0 = m_Seg.m_vP0;
				CDistancePointRec<T> queryPR(m_vClosestPoint0, m_Rec);
				sqrDist = queryPR.ComputeDistanceSqr();
				m_vClosestPoint1 = queryPR.m_vClosestPoint1;
				m_ParamSegment = -m_Seg.m_Ext;
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