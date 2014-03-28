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
class IntersectorSphereSphere
{
  public:  
  IntersectorSphereSphere(const Sphere<T> &sphere1, const Sphere<T> &sphere2)
  {
    sphere1_ = &sphere1;
    sphere2_ = &sphere2;
  };

  bool Intersection();

  const Sphere<T> *sphere1_;
  const Sphere<T> *sphere2_;
    
};

template<class T>
bool IntersectorSphereSphere<T>::Intersection()
{
  
  CVector3<T> vVec = CVector3<T>::createVector(sphere1_->m_vCenter,sphere2_->m_vCenter);
  
  if(vVec.mag() <= sphere1_->m_Rad + sphere2_->m_Rad)
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
