/***************************************************************************
 *   Copyright (C) 2006-2010 by Raphael Muenster   *
 *   raphael@Cortez   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <string>
#include <aabb3.h>
#include <iostream>
#include <genericloader.h>
#include <unstructuredgrid.h>
#include <distops3.h>
#include <triangulator.h>
#include <iomanip>
#include <sstream>
#include <intersectorray3tri3.h>
#include <vtkwriter.h>
#include <world.h>
#include <particlefactory.h>
#include <rigidbodymotionmesh.h>
#include <mymath.h>
#include <distancetriangle.h>
#include <boundingvolumetree3.h>
#include <subdivisioncreator.h>
#include <traits.h>
#include <boundarybox.h>
#include <timecontrol.h>
#include <matrixnxn.h>
#include <vectorn.h>
#include <linearsolvergauss.h>
#include <cppinterface.h>
#include <rigidbodyio.h>
#include <meshobject.h>
#include <vtkwriter.h>
#include <distancemeshpoint.h>

using namespace i3d;

CWorld g_World;
CParticleFactory factory;

int main()
{

  g_World = factory.ProduceMesh("meshes/aneurysma3.obj");
  CRigidBody *body = g_World.m_vRigidBodies[0];

  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->m_pShape);
  pMeshObject->SetFileName("meshes/aneurysma3.obj");
  body->m_dVolume   = body->m_pShape->Volume();
  body->m_dInvMass  = 0.0;

  pMeshObject->m_Model.GenerateBoundingBox();
  pMeshObject->m_Model.GetBox();
  for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
  {
    pMeshObject->m_Model.m_vMeshes[i].GenerateBoundingBox();
  }

  std::vector<CTriangle3r> pTriangles = pMeshObject->m_Model.GenTriangleVector();
  //std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
  CSubDivRessources myRessources(1,5,0,pMeshObject->m_Model.GetBox(),&pTriangles);
  CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
  pMeshObject->m_BVH.InitTree(&subdivider);
  
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObject->m_BVH,VECTOR3(0,0,0));
  Real ddist = distMeshPoint.ComputeDistance();
  printf("Distance: %f \n",ddist);
  printf("ClosestPoint: %f %f %f\n",distMeshPoint.m_Res.m_vClosestPoint.x,
                                    distMeshPoint.m_Res.m_vClosestPoint.y,
                                    distMeshPoint.m_Res.m_vClosestPoint.z);
  
  CVtkWriter writer;
  writer.WriteModel(pMeshObject->m_Model,"output/model.vtk");
  
   
  return 1;
}
