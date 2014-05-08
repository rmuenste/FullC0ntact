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
void CContactGeneratorBoxCylinder<T>::GenerateContactPoints(const Shape<T> &shape0, const Shape<T> &shape1, CSimplexDescriptorGjk<T> &simplex,
                                                            const Transformation<T> &transform0, const Transformation<T> &transform1,
                                                            const Vector3<T> &closestPoint0, const Vector3<T> &closestPoint1,
                                                            Vector3<T> &normal, int &nContacts, std::vector<Vector3<T> > &vContacts)
{
  int i;
  unsigned int iregion,iRegionType;
  unsigned int region1[3];
  const T parallelTolerance = T(1.0)-E1;
  const T perpendicularTolerance = 0.05;

  //the cylinder is the first shape by convention
  const Cylinder<T> &cylinder = dynamic_cast<const Cylinder<T> &>(shape0);
  //the box is the second shape by convention
  const OBB3<T> &box = dynamic_cast<const OBB3<T> &>(shape1);
  
  T dist = (closestPoint0-closestPoint1).mag();
  
  //transform cylinder to box coordinate system
  //get the closest feature of the box
  //transform closest points to box coordinate system
  Vector3<T> centerLocal;
  Matrix3x3<T> matBasis1 = transform1.getMatrix().GetTransposedMatrix();
  Vector3<T> v1Local = closestPoint1 - transform1.getOrigin();
  v1Local = matBasis1 * v1Local; 
  Vector3<T> v0Local = closestPoint0 - transform1.getOrigin();
  v0Local = matBasis1 * v0Local; 

  //transform cylinder to world coordinates
  Vector3<T> uworld = transform0.getMatrix() * cylinder.getU();
  Vector3<T> ulocal = matBasis1 * uworld;
  Vector3<T> centerWorld = transform0.getMatrix() * cylinder.getCenter();
  centerWorld += transform0.getOrigin();

  //compute the line segment through cylinder and transform
  Vector3<T> lineSegment[2];
  lineSegment[1] = (centerWorld+cylinder.getHalfLength()*uworld) - transform1.getOrigin();
  lineSegment[1] = matBasis1 * lineSegment[1];
  lineSegment[0] = (centerWorld-cylinder.getHalfLength()*uworld) - transform1.getOrigin();
  lineSegment[0] = matBasis1 * lineSegment[0];

  for(i=0;i<2;i++)
  {
    region1[i] = box.classifyVertex(lineSegment[i]);
  }

  //bitwise and of the vertices
  iregion  = region1[0];
  iregion &= region1[1];

  //get the region of the box that the final simplex vertices
  //are located in
  iRegionType = box.getRegionType(iregion);
  if(iRegionType==FACE)
  {
    //std::cout<<"configuration: FACE"<<std::endl;
    //get the normal of the box face
    Vector3<T> vFaceNormal = box.getFaceNormal(iregion);
    T dotUF = ulocal * vFaceNormal;
    
    //if the u-axis of the cylinder and face normal are
    //perpendicular then we have a face-edge configuration
    if(fabs(dotUF) < perpendicularTolerance)
    {
      //get the face of the box
      Rectangle3<T> rec = box.getRegionFace(iregion);
      Vector3<T> seg[2];
      Vector3<T> rectangle[4];
      rec.computeVertices(rectangle);
      centerLocal = centerWorld - transform1.getOrigin();
      centerLocal = matBasis1 * centerLocal;
      
      //project the end points of the segment onto the plane defined
      //by the face of the box
      CIntersectorTools<T>::ProjectLineOnBoxPlane(box,iregion,
                            centerLocal+cylinder.getHalfLength()*ulocal,
                            centerLocal-cylinder.getHalfLength()*ulocal,
                            seg);

      //compute the intersection of the edge and the face
      //to get the contact points
      CIntersectorTools<T>::SegmentRectanglePlanar(seg,rectangle,nContacts, vContacts);

      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i]+= (dist/2.0) * vFaceNormal;        
        vContacts[i] = transform1.getMatrix() * vContacts[i] + transform1.getOrigin();
      }
    }
    //if the u-axis of the cylinder and the face normal
    //are parallel then we have a face-face configuration
    else if(fabs(dotUF) > parallelTolerance)
    {
      //get the face of the box
      Rectangle3<T> rec = box.getRegionFace(iregion);
      Vector3<T> circleCenter;
      centerLocal = centerWorld - transform1.getOrigin();
      centerLocal = matBasis1 * centerLocal;
      
      //project the center of the circle onto the face defined by the
      //face of the box
      CIntersectorTools<T>::ProjectPointOnBoxPlane(box,iregion,centerLocal,circleCenter);

      //clip the circular face and the rectangular face to get
      //the contact manifold
      CIntersectorTools<T>::ClipCircleRectangle(rec,circleCenter,cylinder.getRadius(),vContacts);

      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i]+= (dist/2.0) * vFaceNormal;
        vContacts[i] = transform1.getMatrix() * vContacts[i] + transform1.getOrigin();
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
      Segment3<T> seg = box.getRegionEdge(iregion);
      T dotUF = ulocal * seg.dir_;
    
      //check if the u-axis is parallel to the
      //faces connected to the edge
      centerLocal = centerWorld - transform1.getOrigin();
      centerLocal = matBasis1 * centerLocal;
    
      //this can be an edge-face configuration
      unsigned int faces[2];
      int index=-1;
      box.getFacesAtEdge(iregion,faces);
      Vector3<T> vFaceNormal;
      
      //check if the u-axis is parallel to the
      //faces connected to the edge
      for(int j=0;j<2;j++)
      {
         vFaceNormal = box.getFaceNormal(faces[j]);
        if(ulocal * vFaceNormal > parallelTolerance)
        {
          index=j;
        }
      }

    //normal is parallel to one of the faces connected to the edge
    if(index >= 0)
    {
      Rectangle3<T> rec = box.getRegionFace(faces[index]);
      Vector3<T> circleCenter;
      
      //transform the center of the cylinder to the box local coordinate system
      centerLocal = centerWorld - transform1.getOrigin();
      centerLocal = matBasis1 * centerLocal;
      
      //project the center of the circle onto the face defined by the
      //face of the box
      CIntersectorTools<T>::ProjectPointOnBoxPlane(box,faces[index],centerLocal,circleCenter);
      
      CIntersectorTools<T>::ClipCircleRectangle(rec,circleCenter,cylinder.getRadius(),vContacts);

      vFaceNormal = box.getFaceNormal(faces[index]);
      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i]+= (dist/2.0) * vFaceNormal;
        vContacts[i] = transform1.getMatrix() * vContacts[i] + transform1.getOrigin();
      }      
    }//end if
    //if the edge is perpendicular to the u-axis
    else if(fabs(dotUF) < perpendicularTolerance)
    {
      
      //this can be an edge-face configuration
      //intersection sphere segment
      Vector3<T> vLocalNormal = matBasis1 * normal;
      T dotUN = vLocalNormal * ulocal;
      if(fabs(dotUN) > parallelTolerance)
      {
        T projLength = seg.dir_ * centerLocal;
        Vector3<T> projC = seg.center_ + projLength * seg.dir_;
        Sphere<T> sphere(projC,cylinder.getRadius());
        IntersectorSphereSegment<T> sphereSegment(sphere,seg);
        sphereSegment.intersection();
        
        //transform the contact points to world coordinates
        for(int k=0;k<sphereSegment.numIntersections_;k++)
        {
          sphereSegment.points_[k]+= (dist/2.0) * vLocalNormal;                
          vContacts.push_back(transform1.getMatrix() * sphereSegment.points_[k] + transform1.getOrigin());
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
      T projLength = seg.dir_ * centerLocal;
      Vector3<T> projC = seg.center_ + projLength * seg.dir_;
      Vector3<T> seg0[2];
      Vector3<T> seg1[2];

      seg0[0]=seg.center_ + seg.ext_*seg.dir_;
      seg0[1]=seg.center_ - seg.ext_*seg.dir_;

      seg1[0]=projC + cylinder.getHalfLength() * seg.dir_;
      seg1[1]=projC - cylinder.getHalfLength() * seg.dir_;

      //compute contact points
      CIntersectorTools<T>::FindSegmentSegment(seg0,seg1,nContacts,vContacts);

      //transform to world space
      for(int i=0;i<nContacts;i++)
      {
        vContacts[i] = (transform1.getMatrix() * vContacts[i]) + transform1.getOrigin();
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
void CContactGeneratorBoxCylinder<T>::GenerateContactPoints2(const Shape<T> &shape0, const Shape<T> &shape1, CSimplexDescriptorGjk<T> &simplex,
                                                               const Transformation<T> &transform0, const Transformation<T> &transform1,
                                                               const Vector3<T> &closestPoint0, const Vector3<T> &closestPoint1,
                                                               Vector3<T> &normal, int &nContacts, std::vector<Vector3<T> > &vContacts)
{
  int i;
  unsigned int iregion,iRegionType;
  const T parallelTolerance = T(1.0)-E1;
  const T perpendicularTolerance = 0.05;

  //the cylinder is the first shape by convention
  const Cylinder<T> &cylinder = dynamic_cast<const Cylinder<T> &>(shape0);
  //the box is the second shape by convention
  const OBB3<T> &box = dynamic_cast<const OBB3<T> &>(shape1);
  
  T dist = (closestPoint0-closestPoint1).mag();
  //transform cylinder to box coordinate system
  //get the closest feature of the box
  Matrix3x3<T> matBasis1 = transform1.getMatrix().GetTransposedMatrix();
  Vector3<T> v1Local = closestPoint1 - transform1.getOrigin();
  v1Local = matBasis1 * v1Local; 
  Vector3<T> v0Local = closestPoint0 - transform1.getOrigin();
  v0Local = matBasis1 * v0Local; 
  Vector3<T> v1SuppLocal[3];
  unsigned int region1[3];
  unsigned int iregion0;
  unsigned int iregion1;

  Matrix3x3<T> matBasis0 = transform0.getMatrix().GetTransposedMatrix();
  Vector3<T> v1Local0 = closestPoint1 - transform0.getOrigin();
  v1Local0 = matBasis0 * v1Local0; 

  T projL10 = cylinder.getU() * v1Local0;
  if(fabs(projL10) < cylinder.getHalfLength())
  {
    iregion1=1;
  }

  int vertexCount = simplex.GetVertexCount();
  //test the position of the simplex vertices
  //relative to the box
  for(i=0;i<vertexCount;i++)
  {
    v1SuppLocal[i] = simplex.getSupportB(i) - transform1.getOrigin();
    v1SuppLocal[i] = matBasis1 * v1SuppLocal[i];
    region1[i] = box.classifyVertexOnSurface(v1SuppLocal[i]);
  }

  iregion0 = box.classifyVertex(v0Local);

  //bitwise and of the vertices
  iregion = region1[0];
  for(i=1;i<vertexCount;i++)
    iregion&=region1[i];

  //get the region of the box that the final simplex vertices
  //are located in
  iRegionType = box.getRegionType(iregion);
  iRegionType = box.getRegionType(iregion0);

  //the vertices of the simplex are a face
  if(iRegionType==FACE)
  {
    //get the normal of the box face
    Vector3<T> vNormal = box.getFaceNormal(iregion);

    //test if u*face normal == 0 or ==1
    Vector3<T> centerLocal = transform0.getOrigin() - transform1.getOrigin();
    centerLocal = matBasis1 * centerLocal;
    Vector3<T> ulocal = (matBasis1 * transform0.getMatrix()) * cylinder.getU();
    T dotUF = ulocal * vNormal;

    //if the u-axis of the cylinder and the face normal
    //are parallel then we have a face-face configuration
    if(fabs(dotUF) > parallelTolerance)
    {
      //get the face of the box
      Rectangle3<T> rec = box.getRegionFace(iregion);
      Vector3<T> circleCenter;

      //project the center of the circle onto the face defined by the
      //face of the box
      CIntersectorTools<T>::ProjectPointOnBoxPlane(box,iregion,centerLocal,circleCenter);

      //clip the circular face and the rectangular face to get
      //the contact manifold
      CIntersectorTools<T>::ClipCircleRectangle(rec,circleCenter,cylinder.getRadius(),vContacts);

      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i]+= (dist/2.0) * vNormal;
        vContacts[i] = transform1.getMatrix() * vContacts[i] + transform1.getOrigin();
      }
    }
    //if the u-axis of the cylinder and face normal are
    //perpendicular then we have a face-edge configuration
    else if(fabs(dotUF) < perpendicularTolerance)
    {
      //get the face of the box
      Rectangle3<T> rec = box.getRegionFace(iregion);

      Vector3<T> seg[2];
      Vector3<T> rectangle[4];
      rec.computeVertices(rectangle);

      //project the end points of the segment onto the plane defined
      //by the face of the box
      CIntersectorTools<T>::ProjectLineOnBoxPlane(box,iregion,
                            centerLocal+cylinder.getHalfLength()*ulocal,
                            centerLocal-cylinder.getHalfLength()*ulocal,
                            seg);

      //compute the intersection of the edge and the face
      //to get the contact points
      CIntersectorTools<T>::SegmentRectanglePlanar(seg,rectangle,nContacts, vContacts);

      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i]+= (dist/2.0) * vNormal;        
        vContacts[i] = transform1.getMatrix() * vContacts[i] + transform1.getOrigin();
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
    Segment3<T> seg = box.getRegionEdge(iregion);
    
    //transform the cylinder to the box coordinate system
    Vector3<T> centerLocal = transform0.getOrigin() - transform1.getOrigin();
    centerLocal = matBasis1 * centerLocal;
    Vector3<T> ulocal = matBasis1 * cylinder.getU();
    T dotUF = ulocal * seg.dir_;
    
    //if the edge is perpendicular to the u-axis
    if(fabs(dotUF) < perpendicularTolerance)
    {
      //this can be an edge-face configuration
      unsigned int faces[2];
      int index=-1;
      box.getFacesAtEdge(iregion,faces);
      Rectangle3<T> rec;

      //check if the u-axis is parallel to the
      //faces connected to the edge
      for(int j=0;j<2;j++)
      {
        Vector3<T> vNormal = box.getFaceNormal(faces[j]);
        if(ulocal * vNormal > parallelTolerance)
        {
          index=j;
        }
      }

      //if we found a parallel face then
      //we have a face-face configuration
      if(index >= 0)
        rec=box.getRegionFace(faces[index]);
      
      Vector3<T> circleCenter;
      CIntersectorTools<T>::ProjectPointOnBoxPlane(box,faces[index],centerLocal,circleCenter);
      CIntersectorTools<T>::ClipCircleRectangle(rec,circleCenter,cylinder.getRadius(),vContacts);
      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i] = transform1.getMatrix() * vContacts[i] + transform1.getOrigin();
      }
    }
    //if the edge is parallel to the u-axis
    else if(fabs(dotUF) > parallelTolerance)
    {
      //segment-segment
      //we know the u-axis is parallel to the edge
      //project the center of the cylinder onto the edge
      T projLength = seg.dir_ * centerLocal;
      Vector3<T> projC = seg.center_ + projLength * seg.dir_;
      Vector3<T> seg0[2];
      Vector3<T> seg1[2];

      seg0[0]=seg.center_ + seg.ext_*seg.dir_;
      seg0[1]=seg.center_ - seg.ext_*seg.dir_;

      seg1[0]=projC + cylinder.getHalfLength() * seg.dir_;
      seg1[1]=projC - cylinder.getHalfLength() * seg.dir_;

      //compute contact points
      CIntersectorTools<T>::FindSegmentSegment(seg0,seg1,nContacts,vContacts);

      //transform to world space
      for(int i=0;i<nContacts;i++)
      {
        vContacts[i] = (transform1.getMatrix() * vContacts[i]) + transform1.getOrigin();
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
