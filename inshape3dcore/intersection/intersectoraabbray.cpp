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

#include "intersectoraabbray.h"

namespace i3d {

template<class T>
CIntersectorAABBRay3<T>::CIntersectorAABBRay3(const Ray3<T> &rRay, const AABB3<T> &bxAABB) : m_rRay(&rRay), m_bxAABB3(&bxAABB)
{

}//end constructor

template<class T>
bool CIntersectorAABBRay3<T>::Intersection()
{

	T t1,t2;
	T tNear,tFar;
	tNear = -std::numeric_limits<T>::max();
	tFar =  std::numeric_limits<T>::max();

	T xd = m_rRay->m_vDir.x;
	T yd = m_rRay->m_vDir.y;
	T zd = m_rRay->m_vDir.z;

	T x0 = m_rRay->m_vOrig.x;
	T y0 = m_rRay->m_vOrig.y;
	T z0 = m_rRay->m_vOrig.z;

	T x1 = m_bxAABB3->xmin();
	T xh = m_bxAABB3->xmax();

	T y1 = m_bxAABB3->ymin();
	T yh = m_bxAABB3->ymax();

	T z1 = m_bxAABB3->zmin();
	T zh = m_bxAABB3->zmax();


	//ray parallel to the planes
	if(fabs(m_rRay->m_vDir.x) < E5)
	{
		//origin is not between the planes
		if(m_rRay->m_vOrig.x < m_bxAABB3->xmin() || m_rRay->m_vOrig.x > m_bxAABB3->xmax())
		{
			return false;
		}//end if

	}//end if
	//ray not parallel to the planes, calculate the intersection
	else
	{

		//intersection with minX plane
		//x0 + t * xd = x1 so we get:
		t1 = (x1 - x0)/xd;

		//intersection with maX plane
		//x0 + t * xd = xh so we get:
		t2 = (xh - x0)/xd;

		//swap if t1 > t2
		if(t1 > t2)
		{
			T temp = t1;
			t1 = t2;
			t2 = temp;
		}//end if

		//value of tNear
		if(t1 > tNear) tNear = t1;

		//value of tFar
		if(t2 < tFar) tFar = t2;

		//check if the ray misses the box
		if(tNear > tFar) return false;

		//check if the ray is behind the box
		if(tFar < 0) return false;


	}//end else

	tNear = -std::numeric_limits<T>::max();
	tFar =  std::numeric_limits<T>::max();

	//ray parallel to y planes
	if(fabs(m_rRay->m_vDir.y) < E5)
	{
		//origin is not between the planes
		if(m_rRay->m_vOrig.y < m_bxAABB3->ymin() || m_rRay->m_vOrig.y > m_bxAABB3->ymax())
		{
			return false;
		}//end if
	}//end if
	//ray not parallel to the planes, calculate the intersection
	else
	{

		//intersection with minY plane
		//y0 + t * yd = y1 so we get:
		t1 = (y1 - y0)/yd;

		//intersection with maX plane
		//y0 + t * yd = yh so we get:
		t2 = (yh - y0)/yd;

		//swap if t1 > t2
		if(t1 > t2)
		{
			T temp = t1;
			t1 = t2;
			t2 = temp;
		}//end if

		//value of tNear
		if(t1 > tNear) tNear = t1;

		//value of tFar
		if(t2 < tFar) tFar = t2;


		//check if the ray misses the box
		if(tNear > tFar) return false;

		//check if the ray is behind the box
		if(tFar < 0) return false;

	}//end else

	tNear = -std::numeric_limits<T>::max();
	tFar =  std::numeric_limits<T>::max();

	//ray parallel to z planes
	if(fabs(m_rRay->m_vDir.z) < E5)
	{
		//origin is not between the planes
		if(m_rRay->m_vOrig.z < m_bxAABB3->zmin() || m_rRay->m_vOrig.z > m_bxAABB3->zmax())
		{
			return false;
		}//end if

	}//end if
	//ray not parallel to the planes, calculate the intersections
	else
	{

		//intersection with minX plane
		//z0 + t * zd = z1 so we get:
		t1 = (z1 - z0)/zd;

		//intersection with maX plane
		//z0 + t * zd = zh so we get:
		t2 = (zh - z0)/zd;

		//swap if t1 > t2
		if(t1 > t2)
		{
			T temp = t1;
			t1 = t2;
			t2 = temp;
		}//end if

		//value of tNear
		if(t1 > tNear) tNear = t1;

		//value of tFar
		if(t2 < tFar) tFar = t2;

		//check if the ray misses the box
		if(tNear > tFar) return false;

		//check if the ray is behind the box
		if(tFar < 0) return false;

	}//End else

	//all tests were survived, so there is an intersection
    return true;

}//end Intersection

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template
class CIntersectorAABBRay3<Real>;

//----------------------------------------------------------------------------
}

