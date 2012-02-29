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
#include "contactgeneratorboxcylinder.h"
#include <intersectortools.h>
#include <intersectorspheresegment.h>

namespace i3d {

template<class T>
CContactGeneratorBoxCylinder<T>::CContactGeneratorBoxCylinder() 
{

}

template<class T>
CContactGeneratorBoxCylinder<T>::~CContactGeneratorBoxCylinder() 
{

}

template<class T>
void CContactGeneratorBoxCylinder<T>::GenerateContactPoints(const CShape<T> &shape0, const CShape<T> &shape1, CSimplexDescriptorGjk<T> &simplex,
                                                            const CTransform<T> &transform0, const CTransform<T> &transform1,
                                                            const CVector3<T> &closestPoint0, const CVector3<T> &closestPoint1,
                                                            CVector3<T> &normal, int &nContacts, std::vector<CVector3<T> > &vContacts)
{
  int i;
  unsigned int iregion,iRegionType;
  unsigned int region1[3];
  const T parallelTolerance = T(1.0)-E1;
  const T perpendicularTolerance = 0.05;

  //the cylinder is the first shape by convention
  const CCylinder<T> &cylinder = dynamic_cast<const CCylinder<T> &>(shape0);
  //the box is the second shape by convention
  const COBB3<T> &box = dynamic_cast<const COBB3<T> &>(shape1);
  
  T dist = (closestPoint0-closestPoint1).mag();
  
  //transform cylinder to box coordinate system
  //get the closest feature of the box
  //transform closest points to box coordinate system
  CVector3<T> centerLocal;
  CMatrix3x3<T> matBasis1 = transform1.GetTransformation().GetTransposedMatrix();
  CVector3<T> v1Local = closestPoint1 - transform1.GetOrigin();
  v1Local = matBasis1 * v1Local; 
  CVector3<T> v0Local = closestPoint0 - transform1.GetOrigin();
  v0Local = matBasis1 * v0Local; 

  //transform cylinder to world coordinates
  CVector3<T> uworld = transform0.GetTransformation() * cylinder.GetU();
  CVector3<T> ulocal = matBasis1 * uworld;
  CVector3<T> centerWorld = transform0.GetTransformation() * cylinder.GetCenter();
  centerWorld += transform0.GetOrigin();

  //compute the line segment through cylinder and transform
  CVector3<T> lineSegment[2];
  lineSegment[1] = (centerWorld+cylinder.GetHalfLength()*uworld) - transform1.GetOrigin();
  lineSegment[1] = matBasis1 * lineSegment[1];
  lineSegment[0] = (centerWorld-cylinder.GetHalfLength()*uworld) - transform1.GetOrigin();
  lineSegment[0] = matBasis1 * lineSegment[0];

  for(i=0;i<2;i++)
  {
    region1[i] = box.ClassifyVertex(lineSegment[i]);
  }

  //bitwise and of the vertices
  iregion  = region1[0];
  iregion &= region1[1];

  //get the region of the box that the final simplex vertices
  //are located in
  iRegionType = box.GetRegionType(iregion);
  if(iRegionType==FACE)
  {
    //std::cout<<"configuration: FACE"<<std::endl;
    //get the normal of the box face
    CVector3<T> vFaceNormal = box.GetFaceNormal(iregion);
    T dotUF = ulocal * vFaceNormal;
    
    //if the u-axis of the cylinder and face normal are
    //perpendicular then we have a face-edge configuration
    if(fabs(dotUF) < perpendicularTolerance)
    {
      //get the face of the box
      CRectangle3<T> rec = box.GetRegionFace(iregion);
      CVector3<T> seg[2];
      CVector3<T> rectangle[4];
      rec.ComputeVertices(rectangle);
      centerLocal = centerWorld - transform1.GetOrigin();
      centerLocal = matBasis1 * centerLocal;
      
      //project the end points of the segment onto the plane defined
      //by the face of the box
      CIntersectorTools<T>::ProjectLineOnBoxPlane(box,iregion,
                            centerLocal+cylinder.GetHalfLength()*ulocal,
                            centerLocal-cylinder.GetHalfLength()*ulocal,
                            seg);

      //compute the intersection of the edge and the face
      //to get the contact points
      CIntersectorTools<T>::SegmentRectanglePlanar(seg,rectangle,nContacts, vContacts);

      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i]+= (dist/2.0) * vFaceNormal;        
        vContacts[i] = transform1.GetTransformation() * vContacts[i] + transform1.GetOrigin();
      }
    }
    //if the u-axis of the cylinder and the face normal
    //are parallel then we have a face-face configuration
    else if(fabs(dotUF) > parallelTolerance)
    {
      //get the face of the box
      CRectangle3<T> rec = box.GetRegionFace(iregion);
      CVector3<T> circleCenter;
      centerLocal = centerWorld - transform1.GetOrigin();
      centerLocal = matBasis1 * centerLocal;
      
      //project the center of the circle onto the face defined by the
      //face of the box
      CIntersectorTools<T>::ProjectPointOnBoxPlane(box,iregion,centerLocal,circleCenter);

      //clip the circular face and the rectangular face to get
      //the contact manifold
      CIntersectorTools<T>::ClipCircleRectangle(rec,circleCenter,cylinder.GetRadius(),vContacts);

      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i]+= (dist/2.0) * vFaceNormal;
        vContacts[i] = transform1.GetTransformation() * vContacts[i] + transform1.GetOrigin();
      }
    }
    //in all other cases one contact point is sufficient
    else
    {
      //only one contact point and exit
      vContacts.push_back(T(0.5) * (closestPoint1 + closestPoint0));
    }
  }
  else if(iRegionType==EDGE)
  {
      //std::cout<<"configuration: EDGE"<<std::endl;    
      CSegment3<T> seg = box.GetRegionEdge(iregion);
      T dotUF = ulocal * seg.m_vDir;
    
      //check if the u-axis is parallel to the
      //faces connected to the edge
      centerLocal = centerWorld - transform1.GetOrigin();
      centerLocal = matBasis1 * centerLocal;
    
      //this can be an edge-face configuration
      unsigned int faces[2];
      int index=-1;
      box.GetFacesAtEdge(iregion,faces);
      CVector3<T> vFaceNormal;
      
      //check if the u-axis is parallel to the
      //faces connected to the edge
      for(int j=0;j<2;j++)
      {
         vFaceNormal = box.GetFaceNormal(faces[j]);
        if(ulocal * vFaceNormal > parallelTolerance)
        {
          index=j;
        }
      }

    //normal is parallel to one of the faces connected to the edge
    if(index >= 0)
    {
      CRectangle3<T> rec = box.GetRegionFace(faces[index]);
      CVector3<T> circleCenter;
      
      //transform the center of the cylinder to the box local coordinate system
      centerLocal = centerWorld - transform1.GetOrigin();
      centerLocal = matBasis1 * centerLocal;
      
      //project the center of the circle onto the face defined by the
      //face of the box
      CIntersectorTools<T>::ProjectPointOnBoxPlane(box,faces[index],centerLocal,circleCenter);
      
      CIntersectorTools<T>::ClipCircleRectangle(rec,circleCenter,cylinder.GetRadius(),vContacts);

      vFaceNormal = box.GetFaceNormal(faces[index]);
      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i]+= (dist/2.0) * vFaceNormal;
        vContacts[i] = transform1.GetTransformation() * vContacts[i] + transform1.GetOrigin();
      }      
    }//end if
    //if the edge is perpendicular to the u-axis
    else if(fabs(dotUF) < perpendicularTolerance)
    {
      
      //this can be an edge-face configuration
      //intersection sphere segment
      CVector3<T> vLocalNormal = matBasis1 * normal;
      T dotUN = vLocalNormal * ulocal;
      if(fabs(dotUN) > parallelTolerance)
      {
        T projLength = seg.m_vDir * centerLocal;
        CVector3<T> projC = seg.m_vCenter + projLength * seg.m_vDir;
        CSphere<T> sphere(projC,cylinder.GetRadius());
        CIntersectorSphereSegment<T> sphereSegment(sphere,seg);
        sphereSegment.Intersection();
        
        //transform the contact points to world coordinates
        for(int k=0;k<sphereSegment.m_iNumIntersections;k++)
        {
          sphereSegment.m_vPoints[k]+= (dist/2.0) * vLocalNormal;                
          vContacts.push_back(transform1.GetTransformation() * sphereSegment.m_vPoints[k] + transform1.GetOrigin());
        }
      }
      else
      {
        vContacts.push_back(T(0.5) * (closestPoint1 + closestPoint0));
      }
    }
    //if the edge is parallel to the u-axis
    else if(fabs(dotUF) > parallelTolerance)
    {
      //segment-segment
      //we know the u-axis is parallel to the edge
      //project the center of the cylinder onto the edge
      T projLength = seg.m_vDir * centerLocal;
      CVector3<T> projC = seg.m_vCenter + projLength * seg.m_vDir;
      CVector3<T> seg0[2];
      CVector3<T> seg1[2];

      seg0[0]=seg.m_vCenter + seg.m_Ext*seg.m_vDir;
      seg0[1]=seg.m_vCenter - seg.m_Ext*seg.m_vDir;

      seg1[0]=projC + cylinder.GetHalfLength() * seg.m_vDir;
      seg1[1]=projC - cylinder.GetHalfLength() * seg.m_vDir;

      //compute contact points
      CIntersectorTools<T>::FindSegmentSegment(seg0,seg1,nContacts,vContacts);

      //transform to world space
      for(int i=0;i<nContacts;i++)
      {
        vContacts[i] = (transform1.GetTransformation() * vContacts[i]) + transform1.GetOrigin();
        vContacts[i]+= (dist/2.0)*normal;
      }
    }
    else
    {
      vContacts.push_back(T(0.5) * (closestPoint1 + closestPoint0));
    }
  }
  else
  {
      //std::cout<<"configuration: VERTEX"<<std::endl;        
      vContacts.push_back(T(0.5) * (closestPoint1 + closestPoint0));    
  }

}

template<class T>
void CContactGeneratorBoxCylinder<T>::GenerateContactPoints2(const CShape<T> &shape0, const CShape<T> &shape1, CSimplexDescriptorGjk<T> &simplex,
                                                               const CTransform<T> &transform0, const CTransform<T> &transform1,
                                                               const CVector3<T> &closestPoint0, const CVector3<T> &closestPoint1,
                                                               CVector3<T> &normal, int &nContacts, std::vector<CVector3<T> > &vContacts)
{
  int i;
  unsigned int iregion,iRegionType;
  const T parallelTolerance = T(1.0)-E1;
  const T perpendicularTolerance = 0.05;

  //the cylinder is the first shape by convention
  const CCylinder<T> &cylinder = dynamic_cast<const CCylinder<T> &>(shape0);
  //the box is the second shape by convention
  const COBB3<T> &box = dynamic_cast<const COBB3<T> &>(shape1);
  
  T dist = (closestPoint0-closestPoint1).mag();
  //transform cylinder to box coordinate system
  //get the closest feature of the box
  CMatrix3x3<T> matBasis1 = transform1.GetTransformation().GetTransposedMatrix();
  CVector3<T> v1Local = closestPoint1 - transform1.GetOrigin();
  v1Local = matBasis1 * v1Local; 
  CVector3<T> v0Local = closestPoint0 - transform1.GetOrigin();
  v0Local = matBasis1 * v0Local; 
  CVector3<T> v1SuppLocal[3];
  unsigned int region1[3];
  unsigned int iregion0;
  unsigned int iregion1;

  CMatrix3x3<T> matBasis0 = transform0.GetTransformation().GetTransposedMatrix();
  CVector3<T> v1Local0 = closestPoint1 - transform0.GetOrigin();
  v1Local0 = matBasis0 * v1Local0; 

  T projL10 = cylinder.GetU() * v1Local0;
  if(fabs(projL10) < cylinder.GetHalfLength())
  {
    iregion1=1;
  }

  int vertexCount = simplex.GetVertexCount();
  //test the position of the simplex vertices
  //relative to the box
  for(i=0;i<vertexCount;i++)
  {
    v1SuppLocal[i] = simplex.GetSupportB(i) - transform1.GetOrigin();
    v1SuppLocal[i] = matBasis1 * v1SuppLocal[i];
    region1[i] = box.ClassifyVertexOnSurface(v1SuppLocal[i]);
  }

  iregion0 = box.ClassifyVertex(v0Local);

  //bitwise and of the vertices
  iregion = region1[0];
  for(i=1;i<vertexCount;i++)
    iregion&=region1[i];

  //get the region of the box that the final simplex vertices
  //are located in
  iRegionType = box.GetRegionType(iregion);
  iRegionType = box.GetRegionType(iregion0);

  //the vertices of the simplex are a face
  if(iRegionType==FACE)
  {
    //get the normal of the box face
    CVector3<T> vNormal = box.GetFaceNormal(iregion);

    //test if u*face normal == 0 or ==1
    CVector3<T> centerLocal = transform0.GetOrigin() - transform1.GetOrigin();
    centerLocal = matBasis1 * centerLocal;
    CVector3<T> ulocal = (matBasis1 * transform0.GetTransformation()) * cylinder.GetU();
    T dotUF = ulocal * vNormal;

    //if the u-axis of the cylinder and the face normal
    //are parallel then we have a face-face configuration
    if(fabs(dotUF) > parallelTolerance)
    {
      //get the face of the box
      CRectangle3<T> rec = box.GetRegionFace(iregion);
      CVector3<T> circleCenter;

      //project the center of the circle onto the face defined by the
      //face of the box
      CIntersectorTools<T>::ProjectPointOnBoxPlane(box,iregion,centerLocal,circleCenter);

      //clip the circular face and the rectangular face to get
      //the contact manifold
      CIntersectorTools<T>::ClipCircleRectangle(rec,circleCenter,cylinder.GetRadius(),vContacts);

      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i]+= (dist/2.0) * vNormal;
        vContacts[i] = transform1.GetTransformation() * vContacts[i] + transform1.GetOrigin();
      }
    }
    //if the u-axis of the cylinder and face normal are
    //perpendicular then we have a face-edge configuration
    else if(fabs(dotUF) < perpendicularTolerance)
    {
      //get the face of the box
      CRectangle3<T> rec = box.GetRegionFace(iregion);

      CVector3<T> seg[2];
      CVector3<T> rectangle[4];
      rec.ComputeVertices(rectangle);

      //project the end points of the segment onto the plane defined
      //by the face of the box
      CIntersectorTools<T>::ProjectLineOnBoxPlane(box,iregion,
                            centerLocal+cylinder.GetHalfLength()*ulocal,
                            centerLocal-cylinder.GetHalfLength()*ulocal,
                            seg);

      //compute the intersection of the edge and the face
      //to get the contact points
      CIntersectorTools<T>::SegmentRectanglePlanar(seg,rectangle,nContacts, vContacts);

      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i]+= (dist/2.0) * vNormal;        
        vContacts[i] = transform1.GetTransformation() * vContacts[i] + transform1.GetOrigin();
      }
    }
    //in all other cases one contact point is sufficient
    else
    {
      //only one contact point and exit
      vContacts.push_back(T(0.5) * (closestPoint1 + closestPoint0));
    }
  }
  //the vertices of the simplex are an edge
  else if(iRegionType==EDGE)
  {
    CSegment3<T> seg = box.GetRegionEdge(iregion);
    
    //transform the cylinder to the box coordinate system
    CVector3<T> centerLocal = transform0.GetOrigin() - transform1.GetOrigin();
    centerLocal = matBasis1 * centerLocal;
    CVector3<T> ulocal = matBasis1 * cylinder.GetU();
    T dotUF = ulocal * seg.m_vDir;
    
    //if the edge is perpendicular to the u-axis
    if(fabs(dotUF) < perpendicularTolerance)
    {
      //this can be an edge-face configuration
      unsigned int faces[2];
      int index=-1;
      box.GetFacesAtEdge(iregion,faces);
      CRectangle3<T> rec;

      //check if the u-axis is parallel to the
      //faces connected to the edge
      for(int j=0;j<2;j++)
      {
        CVector3<T> vNormal = box.GetFaceNormal(faces[j]);
        if(ulocal * vNormal > parallelTolerance)
        {
          index=j;
        }
      }

      //if we found a parallel face then
      //we have a face-face configuration
      if(index >= 0)
        rec=box.GetRegionFace(faces[index]);
      
      CVector3<T> circleCenter;
      CIntersectorTools<T>::ProjectPointOnBoxPlane(box,faces[index],centerLocal,circleCenter);
      CIntersectorTools<T>::ClipCircleRectangle(rec,circleCenter,cylinder.GetRadius(),vContacts);
      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i] = transform1.GetTransformation() * vContacts[i] + transform1.GetOrigin();
      }
    }
    //if the edge is parallel to the u-axis
    else if(fabs(dotUF) > parallelTolerance)
    {
      //segment-segment
      //we know the u-axis is parallel to the edge
      //project the center of the cylinder onto the edge
      T projLength = seg.m_vDir * centerLocal;
      CVector3<T> projC = seg.m_vCenter + projLength * seg.m_vDir;
      CVector3<T> seg0[2];
      CVector3<T> seg1[2];

      seg0[0]=seg.m_vCenter + seg.m_Ext*seg.m_vDir;
      seg0[1]=seg.m_vCenter - seg.m_Ext*seg.m_vDir;

      seg1[0]=projC + cylinder.GetHalfLength() * seg.m_vDir;
      seg1[1]=projC - cylinder.GetHalfLength() * seg.m_vDir;

      //compute contact points
      CIntersectorTools<T>::FindSegmentSegment(seg0,seg1,nContacts,vContacts);

      //transform to world space
      for(int i=0;i<nContacts;i++)
      {
        vContacts[i] = (transform1.GetTransformation() * vContacts[i]) + transform1.GetOrigin();
        vContacts[i]+= (dist/2.0)*normal;
      }
    }
    else
    {
      vContacts.push_back(T(0.5) * (closestPoint1 + closestPoint0));
    }
  }
  //the vertices of the simplex are a vertex
  else if(iRegionType==VERTEX)
  {
    //in this configuration there can be
    //only one contact point
    vContacts.push_back(T(0.5) * (closestPoint1 + closestPoint0));
  }
  else
  {
    //in this configuration there can be
    //only one contact point
    vContacts.push_back(T(0.5) * (closestPoint1 + closestPoint0));
  }
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template
class CContactGeneratorBoxCylinder<Real>;

//----------------------------------------------------------------------------
}
