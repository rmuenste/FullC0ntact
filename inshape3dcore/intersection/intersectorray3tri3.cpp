/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#include "intersectorray3tri3.h"
#include <mymath.h>

namespace i3d {

template<class T>
CIntersectorRay3Tri3<T>::CIntersectorRay3Tri3(const Ray3<T> &rRay, const Triangle3<T> &trTriangle) : m_rRay(&rRay), m_trTriangle(&trTriangle)
{

}//end constructor

template<class T>
bool CIntersectorRay3Tri3<T>::Intersection()
{

   // compute the offset origin, edges, and normal
   CVector3<T> kDiff = m_rRay->m_vOrig - m_trTriangle->m_vV0;
   CVector3<T> kEdge1 = m_trTriangle->m_vV1 - m_trTriangle->m_vV0;
   CVector3<T> kEdge2 = m_trTriangle->m_vV2 - m_trTriangle->m_vV0;
   CVector3<T> kNormal = CVector3<T>::Cross(kEdge1,kEdge2);

    // Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = ray direction,
    // E1 = kEdge1, E2 = kEdge2, N = Cross(E1,E2)) by
    //   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
    //   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
    //   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
    T fDdN = m_rRay->m_vDir * kNormal;
    T fSign;
    if (fDdN > CMath<T>::TOLERANCEZERO) //0.0000005)
    {
        fSign = (T)1.0;
    }
    else if (fDdN < CMath<T>::TOLERANCEZERO)//-0.0000005)
    {
        fSign = (T)-1.0;
        fDdN = -fDdN;
    }
    else
    {
        // Ray and triangle are parallel, call it a "no intersection"
        // even if the ray does intersect.
        return false;
    }

    T fDdQxE2 = (m_rRay->m_vDir * CVector3<T>::Cross(kDiff,kEdge2)) * fSign;

    if (fDdQxE2 >= (T)0.0)
    {
        Real fDdE1xQ = (m_rRay->m_vDir * CVector3<T>::Cross(kEdge1,kDiff)) * fSign;
        if (fDdE1xQ >= (T)0.0)
        {
            if (fDdQxE2 + fDdE1xQ <= fDdN)
            {
                // line intersects triangle, check if ray does
                Real fQdN = (kDiff * kNormal) * -fSign;
                if (fQdN >= (T)0.0)
                {
                    // ray intersects triangle
                    return true;
                }
                // else: t < 0, no intersection
            }
            // else: b1+b2 > 1, no intersection
        }
        // else: b2 < 0, no intersection
    }
    // else: b1 < 0, no intersection

    return false;
	

}//end Intersection

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template
class CIntersectorRay3Tri3<Real>;

//----------------------------------------------------------------------------

}


