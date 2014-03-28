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

#ifndef INTERSECTOR2AABB_H
#define INTERSECTOR2AABB_H

#include <aabb3.h>

namespace i3d {

/**
* @brief Computes whether two AABBs intersect
*
* Computes whether two AABBs intersect
*
*/  
template<class T>
class CIntersector2AABB
{
public:
	CIntersector2AABB(const AABB3<T> &rAABB1, const AABB3<T> &rAABB2);
	~CIntersector2AABB(void){};
	
	/*!
	* We calculate if the given ray intersects with the given aabb
	*/
	bool Intersection();
	void Reinit(const AABB3<T> &rAABB1, const AABB3<T> &rAABB2);
	
	const AABB3<T> *m_pAABB1;
	const AABB3<T> *m_pAABB2;
	
};

template<class T>
CIntersector2AABB<T>::CIntersector2AABB(const AABB3<T> &rAABB1, const AABB3<T> &rAABB2) : m_pAABB1(&rAABB1), m_pAABB2(&rAABB2)
{
	
}

template<class T>
bool CIntersector2AABB<T>::Intersection()
{
  //check x overlap, if not return false
  if(!(((m_pAABB1->vertices_[0].x >= m_pAABB2->vertices_[0].x) && (m_pAABB1->vertices_[0].x <= m_pAABB2->vertices_[1].x))
    || ((m_pAABB2->vertices_[0].x >= m_pAABB1->vertices_[0].x) && (m_pAABB2->vertices_[0].x <= m_pAABB1->vertices_[1].x))))
    return false;

  //check y overlap, if not return false	
  if(!(((m_pAABB1->vertices_[0].y >= m_pAABB2->vertices_[0].y) && (m_pAABB1->vertices_[0].y <= m_pAABB2->vertices_[1].y))
    || ((m_pAABB2->vertices_[0].y >= m_pAABB1->vertices_[0].y) && (m_pAABB2->vertices_[0].y <= m_pAABB1->vertices_[1].y))))
    return false;

  //check z overlap, if not return false	
  if(!(((m_pAABB1->vertices_[0].z >= m_pAABB2->vertices_[0].z) && (m_pAABB1->vertices_[0].z <= m_pAABB2->vertices_[1].z))
    || ((m_pAABB2->vertices_[0].z >= m_pAABB1->vertices_[0].z) && (m_pAABB2->vertices_[0].z <= m_pAABB1->vertices_[1].z))))
    return false;

  //x,y,z dimensions overlap return true
  return true;
}

template<class T>
void CIntersector2AABB<T>::Reinit(const AABB3<T> &rAABB1, const AABB3<T> &rAABB2)
{
  m_pAABB1=&rAABB1;
  m_pAABB2=&rAABB2;
}

}

#endif