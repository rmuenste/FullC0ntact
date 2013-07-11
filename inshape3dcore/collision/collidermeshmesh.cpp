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


#include "collidermeshmesh.h"
#include <triangulator.h>
#include <meshobject.h>
#include <distancetriangletriangle.h>
#include <perftimer.h>
#include <aabb3.h>
#include <distancemeshmesh.h>
#include <distancemap.h>
#include "collisioninfo.h"

namespace i3d {

CColliderMeshMesh::CColliderMeshMesh() 
{

}

CColliderMeshMesh::~CColliderMeshMesh() 
{

}

/**
* @see CCollider::Collide
*
*/  
void CColliderMeshMesh::Collide(std::vector<CContact> &vContacts)
{

  CMeshObject<Real> *pObject1 = dynamic_cast<CMeshObject<Real>* >(m_pBody1->m_pShape);
  
  CDistanceMap<Real> *map0 = m_pBody0->m_Map;
  
  CBoundingVolumeTree3<CAABB3<Real>,Real,CTraits,CSubdivisionCreator> *pBVH = &pObject1->m_BVH;  

  std::cout<<"ColliderMeshMesh: Checking distance map"<<std::endl;
   
  //get all the triangles contained in the root node
  Real mindist = CMath<Real>::MAXREAL;
  VECTOR3 cp0(0,0,0);
  VECTOR3 cp_pre(0,0,0);
  VECTOR3 cp_dm(0,0,0);  
  int tri=-1;
  
  CBoundingVolumeNode3<CAABB3<Real>,Real,CTraits> *pNode = pBVH->GetChild(0);
  int s = pNode->m_Traits.m_vTriangles.size();

  CTransformr World2Model = m_pBody0->GetTransformation();
  MATRIX3X3 Model2World = World2Model.GetMatrix();
  World2Model.Transpose();
      
  for(int k=0;k<pNode->m_Traits.m_vTriangles.size();k++)
  {
    CTriangle3<Real> &tri3 = pNode->m_Traits.m_vTriangles[k];
    VECTOR3 points[3];
    points[0] = tri3.m_vV0;
    points[1] = tri3.m_vV1;        
    points[2] = tri3.m_vV2;              
                  
    for(int l=0;l<3;l++)
    {
      //transform the points into distance map coordinate system        
      VECTOR3 vQuery=points[l];
      vQuery = World2Model.GetMatrix() * (vQuery - World2Model.GetOrigin());        
      std::pair<Real, CVector3<Real> > result;
      result = map0->Query(vQuery);
      if(mindist > result.first)
      {
        mindist=result.first;
        cp0=vQuery;
        //cp0=(Model2World*vQuery)+World2Model.GetOrigin();
        cp_pre=points[l];
        cp_dm=result.second;
        tri=k;
      }
      //add contact point
    }
                
  }//end for k  

  
  std::cout<<"Minimal distance: "<<mindist<<std::endl;
  std::cout<<"Closest point(tranformed): "<<cp0<<std::endl;
  std::cout<<"Closest point(distance map): "<<cp_dm<<std::endl;  
  std::cout<<"Original point: "<<cp_pre<<std::endl;
  std::cout<<"Triangle: "<<tri<<std::endl;
  std::cout<<"Triangle p1: "<<pNode->m_Traits.m_vTriangles[tri].m_vV0;        
  std::cout<<"Triangle p2: "<<pNode->m_Traits.m_vTriangles[tri].m_vV1;        
  std::cout<<"Triangle p3: "<<pNode->m_Traits.m_vTriangles[tri].m_vV2;        
  
}

}
