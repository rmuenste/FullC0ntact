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
#include "distanceconvexconvexgjk.h"
#include <distancepointrec.h>
#include <distancepointseg.h>
#include <triangle3.h>
#include <distancetriangle.h>
#include <vtkwriter.h>
#include <iomanip>
#include <sstream>
#include <intersectormpr.h>

namespace i3d {
  
template <class T>  
CDistanceConvexConvexGjk<T>::CDistanceConvexConvexGjk() 
{

}

template <class T>  
CDistanceConvexConvexGjk<T>::~CDistanceConvexConvexGjk() 
{

}

template <class T>  
CDistanceConvexConvexGjk<T>::CDistanceConvexConvexGjk(const CConvexShape<T> &shape0, 
                                                      const CConvexShape<T> &shape1,
                                                      const CTransform<T> &transform0, 
                                                      const CTransform<T> &transform1)
{
  m_pShape0        = &shape0;
  m_pShape1        = &shape1;
  m_pTransform0 = &transform0;
  m_pTransform1 = &transform1;  
}                                                                                                           

template <class T>  
T CDistanceConvexConvexGjk<T>::ComputeDistance()
{
  return sqrt(ComputeDistanceSqr());
}

template <class T>  
T CDistanceConvexConvexGjk<T>::ComputeDistanceSqr()
{
  
  //convergence criterion
  const T epsilon = 0.000001;
  //maximum number of iterations
  const int maxIter = 1000;
  int iter = 0;
  bool intersecting=true;

  //get a start point on the minkowski difference
  CVector3<T> v = m_pShape0->GetPointOnBoundary() - m_pShape1->GetPointOnBoundary();
  CVector3<T> w;
  
  //get the support vertices
  CVector3<T> suppA = m_pShape0->GetSupport(-v);
  CVector3<T> suppB = m_pShape1->GetSupport(v);

  //calculate the initial w
  w = suppA - suppB;

  //update the bookkeeping
  m_vSimplex.AddSupportA(suppA);
  m_vSimplex.AddSupportB(suppB);
  
  //calculate the squared norm
  T norm2 = v*v;
  
  //test convergence criterion
  while((norm2 - (v*w) > epsilon) && iter < maxIter)
  {

  //Union of W and w
  m_vSimplex.AddSimplexVertex(w);
  m_vSimplex.IncreaseVertexCount();

  std::cout<<"GJK-Iteration: "<<iter<<std::endl;
  std::cout<<"----------------------------"<<std::endl;
  std::cout<<"norm2 - (v*w): "<<norm2 - (v*w)<<std::endl;
  std::cout<<"w vertex: "<<w;
  for(int i=0;i<m_vSimplex.GetVertexCount();i++)
  {
    std::cout<<m_vSimplex.GetSimplexVertex(i);
  }
  std::cout<<"----------------------------"<<std::endl;

  //update the bookkeeping, increase the number of simplex vertices

  //compute the smallest set X such that v=MinDist(Y) is 
  //contained in X, v will be updated
  m_vSimplex = ComputeSmallestSet(m_vSimplex,v);

  //get the vertex closest to the origin in direction -v
  suppA = m_pShape0->GetSupport(-v);
  suppB = m_pShape1->GetSupport(v);
  //calculate w
  w = suppA - suppB;

  //update the bookkeeping
  m_vSimplex.AddSupportA(suppA);
  m_vSimplex.AddSupportB(suppB);

  //squared norm
  norm2 = v*v;

  std::cout<<"distance: "<<sqrt(norm2)<<std::endl;
  std::cout<<"v * w: "<<v*w<<std::endl;

  //T test = v*w - norm2;
  if(v*w > 0)
    intersecting=false;
  else
  {
    std::cout<<"touching/penetration"<<std::endl;
    CIntersectorMPR<T> intersector(*m_pShape0,*m_pShape1);
    intersector.Intersection();
    break;
  }

  CVtkWriter writer;

  std::vector<VECTOR3> verts;
  for(int j=0;j<m_vSimplex.GetVertexCount();j++)
  {
    verts.push_back(m_vSimplex.GetSimplexVertex(j));
  }
  
  std::string sFileName("output/gjk");
  std::ostringstream sName;
  sName<<"."<<std::setfill('0')<<std::setw(3)<<iter<<".vtk";
  sFileName.append(sName.str());
  writer.WriteGJK(verts, iter, sFileName.c_str());

  //increase the iteration count
  iter++;
  }

  //computation of closest points
  ComputeClosestPoints();

  //return the squared distance
  return norm2;
}

template <class T>
CSimplexDescriptorGjk<T> CDistanceConvexConvexGjk<T>::ComputeSmallestSet(CSimplexDescriptorGjk<T> simplex, 
                                               CVector3<T> &v)
{
  int i;
  int n = simplex.GetVertexCount();
  int tableedges[4]={0,1,3,6};
  int edges[][2]={{0,1},{0,2},{1,2},{0,3},{1,3},{2,3}};
  int faces[][3]={{0,1,2},{0,1,3},{0,2,3},{1,2,3}};  
  int nfaces[4]={0,0,1,4};
  CSimplexDescriptorGjk<T> newSimplex;
  
  //brute force
  T minDist=std::numeric_limits<T>::max();
  
  //feature[0] type: 0=vertex,1=edge,2=triangle
  //feature[1] featureindex 0,1,2,3,4 faces or vertices, 0,1,2,3,4,5 edges
  int feature[2];
  
  //for all faces
  for(i=0;i<nfaces[n-1];i++)
  {
    CTriangle3<T> tri(simplex.GetSimplexVertex(faces[i][0]),simplex.GetSimplexVertex(faces[i][1]),simplex.GetSimplexVertex(faces[i][2]));
    CDistancePointTriangle<T> distTriangle(tri,CVector3<T>(0,0,0));
    T dist = distTriangle.ComputeDistance();
    if(dist < minDist)
    {
      minDist = dist;
      feature[0] = 2;
      feature[1] = i;
      v = distTriangle.m_vClosestPoint1;
      newSimplex.m_dBarycentricCoordinates[0]= 1.0 - distTriangle.ds - distTriangle.dt;
      newSimplex.m_dBarycentricCoordinates[1]= distTriangle.ds;
      newSimplex.m_dBarycentricCoordinates[2]= distTriangle.dt;
    }
  }
  
  //for all edges
  for(i=0;i<tableedges[n-1];i++)
  {
    CSegment3<T> seg(simplex.GetSimplexVertex(edges[i][0]),simplex.GetSimplexVertex(edges[i][1]));
    CDistancePointSeg<T> distPointSeg(CVector3<T>(0,0,0),seg);
    T dist = distPointSeg.ComputeDistance();
    if(dist <= minDist)
    {
      minDist = dist;
      feature[0] = 1;
      feature[1] = i;
      v = distPointSeg.m_vClosestPoint1;
      newSimplex.m_dBarycentricCoordinates[1]=(distPointSeg.m_ParamSegment + distPointSeg.m_Seg.m_Ext)/(2*distPointSeg.m_Seg.m_Ext);
      newSimplex.m_dBarycentricCoordinates[0]=1.0 - newSimplex.m_dBarycentricCoordinates[1];
    }
  }

  //for all vertices
  for(i=0;i<n;i++)
  {
    CVector3<T> vertex = simplex.GetSimplexVertex(i);
    T dist = vertex.mag();
    if(dist <= minDist)
    {
      minDist = dist;
      feature[0] = 0;
      feature[1] = i;
      v = vertex;
    }
  }


  //build new simplex and adjust barycentric coordinates
  if(feature[0]==0)
  {
    //vertex case
    newSimplex.SetVertexCount(0);
    newSimplex.AddSimplexVertex(simplex.GetSimplexVertex(feature[1]));
    newSimplex.AddSupportA(simplex.GetSupportA(feature[1]));
    newSimplex.AddSupportB(simplex.GetSupportB(feature[1]));
    newSimplex.m_dBarycentricCoordinates[0]=1.0;
    newSimplex.IncreaseVertexCount();
  }
  else if(feature[0]==1)
  {
    //edge case
    int j  = feature[1];
    int v0 = edges[j][0]; 
    int v1 = edges[j][1]; 

    newSimplex.SetVertexCount(0);
    newSimplex.AddSimplexVertex(simplex.GetSimplexVertex(v0));
    newSimplex.AddSupportA(simplex.GetSupportA(v0));
    newSimplex.AddSupportB(simplex.GetSupportB(v0));
    newSimplex.IncreaseVertexCount();

    newSimplex.AddSimplexVertex(simplex.GetSimplexVertex(v1));
    newSimplex.AddSupportA(simplex.GetSupportA(v1));
    newSimplex.AddSupportB(simplex.GetSupportB(v1));
    newSimplex.IncreaseVertexCount();
  }
  else if(feature[0]==2)
  {
    //face case
    int j=feature[1];
    int v0 = faces[j][0]; 
    int v1 = faces[j][1]; 
    int v2 = faces[j][2]; 


    newSimplex.SetVertexCount(0);
    newSimplex.AddSimplexVertex(simplex.GetSimplexVertex(v0));
    newSimplex.AddSupportA(simplex.GetSupportA(v0));
    newSimplex.AddSupportB(simplex.GetSupportB(v0));
    newSimplex.IncreaseVertexCount();

    newSimplex.AddSimplexVertex(simplex.GetSimplexVertex(v1));
    newSimplex.AddSupportA(simplex.GetSupportA(v1));
    newSimplex.AddSupportB(simplex.GetSupportB(v1));
    newSimplex.IncreaseVertexCount();

    newSimplex.AddSimplexVertex(simplex.GetSimplexVertex(v2));
    newSimplex.AddSupportA(simplex.GetSupportA(v2));
    newSimplex.AddSupportB(simplex.GetSupportB(v2));
    newSimplex.IncreaseVertexCount();
  }

  return newSimplex;
}

template <class T>  
void CDistanceConvexConvexGjk<T>::ComputeClosestPoints()
{
  m_vClosestPoint0 = CVector3<T>(0,0,0);
  m_vClosestPoint1 = CVector3<T>(0,0,0);
  for(int i=0;i<m_vSimplex.GetVertexCount();i++)
  {
    m_vClosestPoint0+=m_vSimplex.GetSupportA(i)*m_vSimplex.m_dBarycentricCoordinates[i];
    m_vClosestPoint1+=m_vSimplex.GetSupportB(i)*m_vSimplex.m_dBarycentricCoordinates[i];
    //std::cout<<"ShapeA: "<<m_vSimplex.GetSupportA(i)<<std::endl;
    //std::cout<<"ShapeB: "<<m_vSimplex.GetSupportB(i)<<std::endl;
  }
  if(m_vSimplex.GetVertexCount()==3)
  {
//    std::cout<<"triangle case"<<std::endl;
  }
  if(m_vSimplex.GetVertexCount()==2)
  {
//    std::cout<<"edge case"<<std::endl;
  }
  if(m_vSimplex.GetVertexCount()==1)
  {
//    std::cout<<"vertex case"<<std::endl;
  }
  //if(m_vSimplex.GetVertexCount()==1)
  //{
  //  m_vClosestPoint0 = m_vSimplex.GetSupportA(0);
  //  m_vClosestPoint1 = m_vSimplex.GetSupportB(0);
  //}
  //else if(m_vSimplex.GetVertexCount()==2)
  //{
  //  CVector3<T> v0   = m_vSimplex.GetSupportA(0);
  //  CVector3<T> vdir = m_vSimplex.GetSupportA(1)-m_vSimplex.GetSupportA(0);
  //  //normalize by hand
  //  T length=vdir.mag();
  //  if(length > CMath<T>::TOLERANCEZERO)
  //    vdir/=length;
  //  m_vClosestPoint0 = v0 + m_vSimplex.m_dBarycentricCoordinates[0] * vdir;

  //  v0   = m_vSimplex.GetSupportB(0);
  //  vdir = m_vSimplex.GetSupportB(1)-m_vSimplex.GetSupportB(0);
  //  //normalize by hand
  //  length=vdir.mag();
  //  if(length > CMath<T>::TOLERANCEZERO)
  //    vdir/=length;
  //  m_vClosestPoint1 = v0 + m_vSimplex.m_dBarycentricCoordinates[1] * vdir;

  //}


}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceConvexConvexGjk<Real>;
//----------------------------------------------------------------------------
}