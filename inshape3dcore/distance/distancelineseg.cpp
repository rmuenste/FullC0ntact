/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#include "distancelineseg.h"

namespace i3d {

template <typename T>
CDistanceLineSeg<T>::CDistanceLineSeg(void)
{
}

template <typename T>
CDistanceLineSeg<T>::CDistanceLineSeg(const Line3<T>& line, const Segment3<T>& seg)
{
	this->m_Line = line;
	this->m_Seg  = seg;
}

template <typename T>
CDistanceLineSeg<T>::~CDistanceLineSeg(void)
{
}

template <typename T>
T CDistanceLineSeg<T>::ComputeDistance()
{
 return (ComputeDistanceSqr());
}

template <typename T>
T CDistanceLineSeg<T>::ComputeDistanceSqr()
{
    Vector3<T> diff = m_Line.origin_ - m_Seg.center_;
    T a01 = -m_Line.dir_ * m_Seg.dir_;
    T b0 = diff * m_Line.dir_;
    T c = diff * diff;
    T det = fabs((T)1 - a01*a01);
    T b1, s0, s1, sqrDist, extDet;

    if (det >= E5)
    {
        // The line and segment are not parallel.
        b1 = -diff * m_Seg.dir_;
        s1 = a01*b0 - b1;
        extDet = m_Seg.ext_*det;

        if (s1 >= -extDet)
        {
            if (s1 <= extDet)
            {
                // Two interior points are closest, one on the line and one
                // on the segment.
                T invDet = ((T)1)/det;
                s0 = (a01*b1 - b0)*invDet;
                s1 *= invDet;
                sqrDist = s0*(s0 + a01*s1 + ((T)2)*b0) +
                    s1*(a01*s0 + s1 + ((T)2)*b1) + c;
            }
            else
            {
                // The endpoint e1 of the segment and an interior point of
                // the line are closest.
                s1 = m_Seg.ext_;
                s0 = -(a01*s1 + b0);
                sqrDist = -s0*s0 + s1*(s1 + ((T)2)*b1) + c;
            }
        }
        else
        {
            // The end point e0 of the segment and an interior point of the
            // line are closest.
            s1 = -m_Seg.ext_;
            s0 = -(a01*s1 + b0);
            sqrDist = -s0*s0 + s1*(s1 + ((T)2)*b1) + c;
        }
    }
    else
    {
        // The line and segment are parallel.  Choose the closest pair so that
        // one point is at segment center.
        s1 = (T)0;
        s0 = -b0;
        sqrDist = b0*s0 + c;
    }

    m_vClosestPoint0 = m_Line.origin_ + m_Line.dir_ * s0;
    m_vClosestPoint1 = m_Seg.center_ + m_Seg.dir_ * s1;
    m_ParamLine = s0;
    m_ParamSegment = s1;

    // Account for numerical round-off errors.
    if (sqrDist < (T)0)
    {
        sqrDist = (T)0;
    }
    return sqrDist;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceLineSeg<float>;

template class CDistanceLineSeg<double>;
//----------------------------------------------------------------------------

}