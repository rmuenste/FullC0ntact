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
    
  //get all the triangles contained in the best node
  Real mindist = CMath<Real>::MAXREAL;
  for(int i=0;i<1;i++)
  {
    CBoundingVolumeNode3<CAABB3<Real>,Real,CTraits> *pNode = pBVH->GetChild(i);
    int s = pNode->m_Traits.m_vTriangles.size();
    for(int k=0;k<pNode->m_Traits.m_vTriangles.size();k++)
    {
      CTriangle3<Real> &tri3 = pNode->m_Traits.m_vTriangles[k];
      VECTOR3 points[3];
      //transform the points into distance map coordinate system

      CTransformr World2Model = m_pBody0->GetTransformation();
      World2Model.Transpose();
      
      points[0] = World2Model.GetMatrix() * (tri3.m_vV0 - World2Model.GetOrigin());
      points[1] = World2Model.GetMatrix() * (tri3.m_vV1 - World2Model.GetOrigin());
      points[2] = World2Model.GetMatrix() * (tri3.m_vV2 - World2Model.GetOrigin());      
                 
      for(int l=0;l<3;l++)
      {
        //check distancemap(points[l])
        //if(dist < eps)
        //add contact point
      }
                  
    }//end for k  
  }//end for i
  
}

}
