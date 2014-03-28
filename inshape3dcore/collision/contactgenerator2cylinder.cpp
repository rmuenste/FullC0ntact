/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

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


//===================================================
//                     INCLUDES
//===================================================
#include "contactgenerator2cylinder.h"
#include <vector2.h>
#include <intersectorspheresphere.h>
#include <intersectorcirclecircle.h>
#include <intersectorspheresegment.h>
#include <cylinder.h>
#include <intersectortools.h>

namespace i3d {

template<class T>
CContactGenerator2Cylinder<T>::CContactGenerator2Cylinder() 
{

}

template<class T>
CContactGenerator2Cylinder<T>::~CContactGenerator2Cylinder() 
{

}

template<class T>
void CContactGenerator2Cylinder<T>::GenerateContactPoints(const Shape<T> &shape0, const Shape<T> &shape1, CSimplexDescriptorGjk<T> &simplex,
                                                               const CTransform<T> &transform0, const CTransform<T> &transform1,
                                                               const CVector3<T> &closestPoint0, const CVector3<T> &closestPoint1,
                                                               CVector3<T> &normal, int &nContacts, std::vector<CVector3<T> > &vContacts)
{
  const T parallelTolerance = T(1.0)-0.0002;
  const T perpendicularTolerance = 0.005;

  const Cylinder<T> &cylinder0 = dynamic_cast<const Cylinder<T>& >(shape0);
  const Cylinder<T> &cylinder1 = dynamic_cast<const Cylinder<T>& >(shape1);

  CVector3<T> closestLocal0;
  CVector3<T> vNormal;

  T dot;

  //distance
  T dist = (closestPoint1-closestPoint0).mag();
  
  //transform the closest point on cylinder1 to cylinder0's coordinate system
  CMatrix3x3<T> matBasis0 = transform0.GetMatrix().GetTransposedMatrix();
  CVector3<T> v0Local = closestPoint1 - transform0.GetOrigin();
  v0Local = matBasis0 * v0Local; 

  //transform the center of cylinder1 to cylinder0's cs
  CVector3<T> c1 = matBasis0 * (transform1.GetOrigin() - transform0.GetOrigin());

  //C1-C0
  CVector3<T> delta = c1-cylinder0.GetCenter();

  //axis of cylinder1 in cylinder0's cs
  CVector3<T> ulocal1 = matBasis0 * (transform1.GetMatrix() * cylinder1.GetU());

  //project v1local0 onto the axis of cylinder0
  T projDelta = cylinder0.GetU() * v0Local;
  T sign = (projDelta < 0) ? -1.0 : 1.0;
  //maybe take away the equality sign here
  if(fabs(projDelta) >= cylinder0.GetHalfLength())
  {
    //v1local is >= the top section or <= the bottom section
    T dotUU = cylinder0.GetU() * ulocal1;
    
    //check for face-face
    //and normal * u > parallelTolerance
    vNormal = matBasis0 * normal;
    T dotUN = vNormal * cylinder0.GetU();
    //the u and the normal need to be parallel for a face-face collision
    if((fabs(dotUU) > parallelTolerance) && fabs(dotUN) > parallelTolerance)
    {
      //intersection circle circle
      //project center of cylinder onto plane
      CVector2<T> circleCenter0 = CVector2<T>(cylinder0.GetCenter().x,cylinder0.GetCenter().y);
      CVector2<T> circleCenter1 = CVector2<T>(c1.x,c1.y);
      CIntersectorCircleCircle<T> intersector(circleCenter0,circleCenter1,
                                           cylinder0.GetRadius(),cylinder1.GetRadius());

      //the normal computed by gjk may be bad due to numerical difficulties, so correct the normal
      normal = (sign > 0) ? transform0.GetMatrix() * -cylinder0.GetU() : transform0.GetMatrix() * cylinder0.GetU();
      if(intersector.Find())
      {
        //transform the contact points to world coordinates
        for(int i=0;i<intersector.m_iNumIntersections;i++)
        {
          CVector3<T> v3d(intersector.m_vPoints[i].x,intersector.m_vPoints[i].y,v0Local.z+(sign * dist/2.0));
          vContacts.push_back(transform0.GetMatrix() * v3d + transform0.GetOrigin());
        }
      }
    }
    //check for a edge-edge collision
    else if((fabs(dotUU) > parallelTolerance) && (fabs(dotUN) < perpendicularTolerance))
    {
      closestLocal0 = closestPoint0 - transform0.GetOrigin();
      closestLocal0 = matBasis0 * closestLocal0;

      CVector3<T> seg0[2];
      CVector3<T> seg1[2];
      CVector3<T> segmentCenter0;
      segmentCenter0.x=closestLocal0.x;
      segmentCenter0.y=closestLocal0.y;
      segmentCenter0.z=0;

      CVector3<T> segmentCenter1;
      segmentCenter1.x=0;
      segmentCenter1.y=0;
      segmentCenter1.z=c1.z;
      
      seg0[0]=cylinder0.GetCenter() + cylinder0.GetHalfLength()*cylinder0.GetU();
      seg0[1]=cylinder0.GetCenter() - cylinder0.GetHalfLength()*cylinder0.GetU();

      //axes are parallel
      seg1[0]=segmentCenter1 + cylinder1.GetHalfLength()*cylinder0.GetU();
      seg1[1]=segmentCenter1 - cylinder1.GetHalfLength()*cylinder0.GetU();

      //compute contact points
      CIntersectorTools<T>::FindSegmentSegment(seg0,seg1,nContacts,vContacts);

      for(int i=0;i<nContacts;i++)
      {
        vContacts[i]-=(cylinder0.GetRadius()+(dist/2.0))*vNormal;
        vContacts[i] = (transform0.GetMatrix() * vContacts[i]) + transform0.GetOrigin();
      }

    }
    //check for face-edge
    else if(fabs(dotUU) < perpendicularTolerance)
    {
      //intersection sphere segment
      CSphere<T> sphere(cylinder0.GetCenter() + projDelta*cylinder0.GetU(),cylinder0.GetRadius());
      CVector3<T> centerSegment(c1.x,c1.y,v0Local.z);
      CSegment3<T> segment(centerSegment,ulocal1,cylinder1.GetHalfLength());
      CIntersectorSphereSegment<T> sphereSegment(sphere,segment);
      sphereSegment.Intersection();
      //transform the contact points to world coordinates
      for(int k=0;k<sphereSegment.m_iNumIntersections;k++)
      {
        sphereSegment.m_vPoints[k]+= (dist/2.0) * cylinder0.GetU();                
        vContacts.push_back(transform0.GetMatrix() * sphereSegment.m_vPoints[k] + transform0.GetOrigin());
      }
    }
    //face-vertex
    else
    {
      //translate contact point along normal
      vContacts.push_back(T(0.5) * (closestPoint1 + closestPoint0));
    }
  }
  else
  {
    //closest point is located in the middle section
    //determine the relative orientation of the u-axes
    T dotUU = cylinder0.GetU() * ulocal1;
    //edge-edge
    if(fabs(dotUU) > parallelTolerance)
    {
      closestLocal0 = closestPoint0 - transform0.GetOrigin();
      closestLocal0 = matBasis0 * closestLocal0;

      CVector3<T> seg0[2];
      CVector3<T> seg1[2];
      seg0[0]=cylinder0.GetCenter() + cylinder0.GetHalfLength()*cylinder0.GetU();
      seg0[1]=cylinder0.GetCenter() - cylinder0.GetHalfLength()*cylinder0.GetU();

      //axes are parallel
      seg1[0]=c1 + cylinder1.GetHalfLength()*cylinder0.GetU();
      seg1[1]=c1 - cylinder1.GetHalfLength()*cylinder0.GetU();

      //compute contact points
      CIntersectorTools<T>::FindSegmentSegment(seg0,seg1,nContacts,vContacts);

      //transform to world space
      for(int i=0;i<nContacts;i++)
      {
        vContacts[i]-=(cylinder0.GetRadius()+(dist/2.0))*vNormal;
        vContacts[i] = (transform0.GetMatrix() * vContacts[i]) + transform0.GetOrigin();
      }
    }
    //face-edge
    //perpendicular and closest0 in top section of
    else if(fabs(dotUU) < perpendicularTolerance)
    {
      //normal = cylinder1.GetU();
      //vNormal = matBasis0 * normal;
      closestLocal0 = closestPoint0 - transform0.GetOrigin();
      closestLocal0 = matBasis0 * closestLocal0; 
      vNormal = closestLocal0 - v0Local;
      
      //TODO:looks instable?? for sure!!
      vNormal /= dist;
      dot = vNormal * ulocal1;
      if(fabs(vNormal * ulocal1) > parallelTolerance)
      {
        //problematic case
        //closestLocal0 = closestPoint0 - transform0.GetOrigin();
        //closestLocal0 = matBasis0 * closestLocal0; 
        //vNormal = closestLocal0 - v0Local;
        ////TODO:looks instable?? for sure!!
        //vNormal /= dist;
        //intersection sphere segment
        CVector3<T> sphereCenter = c1 + (dist+cylinder1.GetHalfLength()) * vNormal;
        CSphere<T> sphere(sphereCenter,cylinder1.GetRadius());
        CVector3<T> centerSegment(closestLocal0.x,closestLocal0.y,0);
        CSegment3<T> segment(centerSegment,cylinder0.GetU(),cylinder0.GetHalfLength());
        CIntersectorSphereSegment<T> sphereSegment(sphere,segment);
        sphereSegment.Intersection();
        //transform the contact points to world coordinates
        for(int k=0;k<sphereSegment.m_iNumIntersections;k++)
        {
          sphereSegment.m_vPoints[k]-= (dist/2.0) * vNormal;                
          vContacts.push_back(transform0.GetMatrix() * sphereSegment.m_vPoints[k] + transform0.GetOrigin());
        }
      }
      else
      {
      //translate contact point along normal
      vContacts.push_back(T(0.5) * (closestPoint1 + closestPoint0));
      }
    }
    //vertex-vertex
    else
    {
      //translate contact point along normal
      vContacts.push_back(T(0.5) * (closestPoint1 + closestPoint0));
    }
  }
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template
class CContactGenerator2Cylinder<Real>;

//----------------------------------------------------------------------------
}
