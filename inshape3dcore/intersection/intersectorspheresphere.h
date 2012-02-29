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

#include <sphere.h>

namespace i3d {

/**
* @brief Computes whether two spheres intersect
*
* Computes whether two spheres intersect
*
*/        
template<class T>
class CIntersectorSphereSphere
{
  public:  
  CIntersectorSphereSphere(const CSphere<T> &Sphere1, const CSphere<T> &Sphere2)
  {
	m_pSphere1 = & Sphere1;
	m_pSphere2 = & Sphere2;
  };

  bool Intersection();

  const CSphere<T> *m_pSphere1;
  const CSphere<T> *m_pSphere2;
  
  
};

template<class T>
bool CIntersectorSphereSphere<T>::Intersection()
{
  
  CVector3<T> vVec = CVector3<T>::createVector(m_pSphere1->m_vCenter,m_pSphere2->m_vCenter);
  
  if(vVec.mag() <= m_pSphere1->m_Rad + m_pSphere2->m_Rad)
  {
	return true;
  }
  else
	return false;
  
}



#ifndef INTERSECTORSPHERESPHERE_H
#define INTERSECTORSPHERESPHERE_H

}

#endif // INTERSECTORSPHERESPHERE_H
