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

#include "distancetriangle.h"
#include <triface.h>

namespace i3d {

template <class T>
CDistancePointTriangle<T>::CDistancePointTriangle(const Triangle3<T> &face,const CVector3<T> &vQuery) 
{
  m_pTriangle = &face;
  m_vQuery    = vQuery;
}

template <class T>
CDistancePointTriangle<T>::~CDistancePointTriangle()
{
}

template <class T>
T CDistancePointTriangle<T>::ComputeDistanceSqr()
{
  T dist = ComputeDistance();
  return dist * dist;
}

template <class T>
T CDistancePointTriangle<T>::ComputeDistance()
{
  T da,db,dc,dd,de,df;
  T ddenom,dnum;
  
  CVector3<T> e0,e1,vClosestPoint;
  
  e0 = m_pTriangle->m_vV1-m_pTriangle->m_vV0;
  e1 = m_pTriangle->m_vV2-m_pTriangle->m_vV0;  
  
  da = e0 * e0;
  db = e0 * e1;
  dc = e1 * e1;
  dd = e0 * (m_pTriangle->m_vV0 - m_vQuery);
  de = e1 * (m_pTriangle->m_vV0 - m_vQuery);  
  
  ddenom = da*dc-db*db;
  ds = db*de-dc*dd;
  dt = db*dd-da*de;
  
  if(ds+dt <= ddenom)
  {
		if(ds < 0)
		{
			if(dt < 0)
			{
				//Region 4
				// Grad(Q) = 2(as+bt+d,bs+ct+e)
				// (0,1)*Grad(Q(0,0)) = (0,1)*(d,e) = e
				// (1,0)*Grad(Q(0,0)) = (1,0)*(d,e) = d
				// min on edge t=0 if (0,1)*Grad(Q(0,0)) < 0 )
				// min on edge s=0 otherwise
				if( de < 0.0)
				{
					ds = ( dd >=0 ? 0 : (-dd >= da ? 1 : dd/da));
					dt = 0;
				}
				else
				{
				ds = 0;
				dt = ( de >=0 ? 0 : (-de >= dc ? 1 : -de/dc));
				}
			}
			//Region 3
			else
			{
				// F(t) = Q(0,t) = ct^2 + 2et + f
				// F’(t)/2 = ct+e
				// F’(T) = 0 when T = -e/c
				ds = 0;
				dt = ( de >=0 ? 0 : (-de >= dc ? 1 : -de/dc));
			}
		}
		//Region 5
		else if(dt < 0)
		{
				// F(s) = Q(s,0) = as^2 + 2ds + f
				// F’(s)/2 = as+d
				// F’(S) = 0 when S = -d/a
				ds = ( dd >=0 ? 0 : (-dd >= da ? 1 : -dd/da));
				dt = 0;
		}
		//Region 0
		else
		{
			T invDenom=T(1.0)/ddenom;
			ds*=invDenom;
			dt*=invDenom;
		}
  }
	else
	{
		//Region 2
		if(ds < 0)
		{
			// Grad(Q) = 2(as+bt+d,bs+ct+e)
			// (0,-1)*Grad(Q(0,1)) = (0,-1)*(b+d,c+e) = -(c+e)
			// (1,-1)*Grad(Q(0,1)) = (1,-1)*(b+d,c+e) = (b+d)-(c+e)
			// min on edge s+t=1 if (1,-1)*Grad(Q(0,1)) < 0 )
			// min on edge s=0 otherwise
			T tmp0 = db + dd;
			T tmp1 = dc + de;
			if( tmp1 > tmp0)
			{
				dnum = tmp1 - tmp0;
				ddenom = da-2*db+dc;
				ds = (dnum >= ddenom ? 1 : dnum/ddenom);
				dt = 1 - ds;
			}
			else
			{
				ds=0;
				dt = (tmp1 <= 0 ? 1 : (de >= 0 ? 0 : -de/dc));
			}
		}
		//Region 6
		else if(dt < 0)
		{
			// Grad(Q) = 2(as+bt+d,bs+ct+e)
			// (-1,0)*Grad(Q(1,0)) = (0,-1)*(b+d,c+e) = -(b+e)
			// (-1,1)*Grad(Q(1,0)) = (1,-1)*(b+d,c+e) = -(a+d)+(b+e)
			// min on edge s+t=1 if (-1,1)*Grad(Q(1,0)) < 0 )
			// min on edge s=0 otherwise
			T tmp0 = -da - dd;
			T tmp1 = db + de;
			T tmp2 = db + dd;
			T tmp3 = dc + de;
			if( tmp1 > tmp0)
			{
				dnum = tmp3 - tmp2;
				ddenom = da-2*db+dc;
				ds = (dnum >= ddenom ? 1 : dnum/ddenom);
				dt = 1 - ds;
			}
			else
			{
				ds = ( dd >=0 ? 0 : (-dd >= da ? 1 : dd/da));
				dt = 0;
			}
		}
		// Region 1
		else
		{
			// F(s) = Q(s,1-s) = (a-2b+c)s^2 + 2(b-c+d-e)s + (c+2e+f)
			// F’(s)/2 = (a-2b+c)s + (b-c+d-e)
			// F’(S) = 0 when S = (c+e-b-d)/(a-2b+c)
			// a-2b+c = |E0-E1|^2 > 0, so only sign of c+e-b-d need be considered
			dnum = (dc+de-db-dd);
			if(dnum <= 0)
			{
				ds=0.0;
			}
			else
			{
				ddenom = da-2*db+dc;
				ds = (dnum >= ddenom ? 1.0 : dnum/ddenom);
			}
			dt = 1 - ds;
		}
	}
	
	m_vClosestPoint0 = m_vQuery;
	m_vClosestPoint1 = m_pTriangle->m_vV0+e0*ds+e1*dt;

	return (m_vClosestPoint0-m_vClosestPoint1).mag();
	
}//end ComputeDistance

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistancePointTriangle<Real>;
//----------------------------------------------------------------------------
}