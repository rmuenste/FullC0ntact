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

#include "collisionpipeline.h"
#include <broadphase.h>
#include <broadphasestrategygrid.h>
#include <broadphasestrategyhgrid.h>
#include <collresponseimpluse.h>
#include <collresponse.h>
#include <world.h>
#include <iostream>
#include <timecontrol.h>
#include <toiestimator.h>
#include <colliderfactory.h>
#include <collresponselcp.h>
#include <perftimer.h>
#include <simplespatialhash.h>
#include <hspatialhash.h>
#include <meshobject.h>
#include <subdivisioncreator.h>
#include <set>

namespace i3d {

CCollisionPipeline::CCollisionPipeline()
{
  m_pWorld     = NULL;
  m_Strategy   = NULL;
  m_Response   = NULL;
  m_BroadPhase = NULL;
  m_iPipelineIterations = 5;
  m_pGraph = new CContactGraph();
  m_pGraph->m_pEdges = new CCollisionHash(1001);
}

void CCollisionPipeline::Init(CWorld *pWorld, int iStrategyId, int iBroadPhase, int iBoundary)
{
	m_pWorld = pWorld;
	m_pTimeControl = pWorld->m_pTimeControl;
  m_iPipelineIterations = 5;

	switch(iStrategyId)
	{
	case CCollisionPipeline::DISCRETECONSTRAINED :
    std::cout<<"DISCRETECONSTRAINED no longer supported"<<std::endl;
    exit(0);
		break;
	case CCollisionPipeline::RIGIDBODY :
  {
		this->m_Strategy = new CBroadPhaseStrategy(m_pWorld,&m_CollInfo);
		m_Response = new CCollResponseLcp(&m_CollInfo,m_pWorld);
		m_Strategy->SetEPS(m_dCollEps);
		m_Response->SetEPS(m_dCollEps);
    CCollResponseLcp *pResponse = dynamic_cast<CCollResponseLcp *>(m_Response);
    pResponse->InitSolverPGS(100,1.0);    
  }
		break;
	case CCollisionPipeline::COMPLEX :
    std::cout<<"Complex no longer supported"<<std::endl;
    exit(0);
		break;
	}

	switch(iBroadPhase)
	{
	case CCollisionPipeline::NAIVE :
		this->m_BroadPhase = new CBroadPhase(m_pWorld,&m_CollInfo,m_Strategy);
		m_BroadPhase->SetEPS(m_dCollEps);
    m_BroadPhase->m_pStrat->m_pImplicitGrid = new CImplicitGrid(new CSimpleSpatialHash(5001),0.05);
		break;
  case CCollisionPipeline::SPATIALHASH :
		this->m_BroadPhase = new CBroadPhase(m_pWorld,&m_CollInfo,m_Strategy);
		m_BroadPhase->SetEPS(m_dCollEps);
    m_BroadPhase->m_pStrat->m_pImplicitGrid = new CImplicitGrid(new CSimpleSpatialHash(5001),0.05);
    break;
  default:
    std::cerr<<"wrong broadphase id in: collisionpipeline.cpp"<<std::endl;
    exit(0);
    break;
	}

	switch(iBoundary)
	{
	case CCollisionPipeline::BOXSHAPED :
		//set up the module that handles the particle-wall
		break;
	case CCollisionPipeline::COMPLEXSHAPED :
		break;
	}

}

void CCollisionPipeline::SetBroadPhaseNaive()
{
  m_Strategy = new CBroadPhaseStrategy(m_pWorld,&m_CollInfo);
  m_Strategy->SetEPS(m_dCollEps);
  m_BroadPhase = new CBroadPhase(m_pWorld,&m_CollInfo,m_Strategy);
  m_BroadPhase->m_BroadPhasePairs = &m_BroadPhasePairs;
  m_BroadPhase->m_pStrat->m_BroadPhasePairs = &m_BroadPhasePairs;  
}

void CCollisionPipeline::SetBroadPhaseSpatialHash()
{
  m_Strategy = new CBroadPhaseStrategyGrid(m_pWorld,&m_CollInfo);
  m_Strategy->SetEPS(m_dCollEps);
  m_BroadPhase = new CBroadPhase(m_pWorld,&m_CollInfo,m_Strategy);
  m_BroadPhase->m_BroadPhasePairs = &m_BroadPhasePairs;
  m_BroadPhase->m_pStrat->m_BroadPhasePairs = &m_BroadPhasePairs;  
  m_BroadPhase->m_pStrat->m_pImplicitGrid = new CImplicitGrid(new CSimpleSpatialHash(5001,0.05,m_pWorld->m_pBoundary->m_Values),0.05);
}

void CCollisionPipeline::SetBroadPhaseHSpatialHash()
{
  m_Strategy = new CBroadPhaseStrategyHGrid(m_pWorld,&m_CollInfo);
  m_Strategy->SetEPS(m_dCollEps);
  m_BroadPhase = new CBroadPhase(m_pWorld,&m_CollInfo,m_Strategy);
  m_BroadPhase->m_BroadPhasePairs = &m_BroadPhasePairs;
  m_BroadPhase->m_pStrat->m_BroadPhasePairs = &m_BroadPhasePairs;  
  m_BroadPhase->m_pStrat->m_pImplicitGrid = new CImplicitGrid(new CHSpatialHash(5001,m_pWorld->m_pBoundary->m_Values,m_pWorld->m_vRigidBodies),0.05);
}

void CCollisionPipeline::Init(CWorld *pWorld, int lcpIterations, int pipelineIterations)
{
  //set the world pointer
  m_pWorld = pWorld;
  m_pTimeControl = pWorld->m_pTimeControl;
  m_Response = new CCollResponseLcp(&m_CollInfo,m_pWorld);
  m_Response->SetEPS(m_dCollEps);
  CCollResponseLcp *pResponse = dynamic_cast<CCollResponseLcp *>(m_Response);
  pResponse->InitSolverPGS(lcpIterations,1.0);    
  m_iPipelineIterations = pipelineIterations;
}

CCollisionPipeline::CCollisionPipeline(const CCollisionPipeline &copy)
{
	m_BroadPhase = copy.m_BroadPhase;
	m_Strategy   = copy.m_Strategy;
	m_pWorld     = copy.m_pWorld;
	m_CollInfo   = copy.m_CollInfo;
}

CCollisionPipeline::~CCollisionPipeline()
{
  if(m_Strategy != NULL)
  {
    delete m_Strategy;
    m_Strategy = NULL;
  }
  if(m_Response != NULL)
  {
    delete m_Response;
    m_Response = NULL;
  }
  if(m_BroadPhase != NULL)
  {
    delete m_BroadPhase;
    m_BroadPhase = NULL;
  }
  delete m_pGraph;   
}

void CCollisionPipeline::SolveCollidingContact()
{
  m_Response->SolveVelocityBased();
}

void CCollisionPipeline::StartPipeline()
{
  vContacts.clear();
  CPerfTimer timer0;
  //#ifdef  PERFTIMINGS
  double dTimeBroad=0.0;
  double dTimeMiddle=0.0;  
  double dTimeNarrow=0.0;
  double dTimeLCP=0.0;
  double dTimeLCPResting=0.0;
  double dTimePostContactAnalysis=0.0;  

  timer0.Start();
  StartBroadPhase();
  dTimeBroad+=timer0.GetTime();  
  timer0.Start();  
  StartMiddlePhase();
  dTimeMiddle+=timer0.GetTime();    

  for(int i=0;i<m_iPipelineIterations;i++)
  {
    timer0.Start();
    StartNarrowPhase();
    dTimeNarrow+=timer0.GetTime();
    //get timings
    timer0.Start();
    SolveCollidingContact();
    //get timings
    dTimeLCP+=timer0.GetTime();
    //UpdateContactGraph();
    m_CollInfo.clear();
  }

  int nContactPoints=0;
  int nEdges=0;
  int nRealEdges=0;

//   CCollisionHash::iterator hiter = m_pGraph->m_pEdges->begin();
//   std::set<CCollisionInfo,CompColl> mySet;
//   if(!m_pGraph->m_pEdges->IsEmpty())
//   {
//     for(;hiter!=m_pGraph->m_pEdges->end();hiter++)
//     {
//       CCollisionInfo &info = *hiter;
//       mySet.insert(info);
//       nEdges++;
//       nContactPoints+=info.m_vContacts.size();
//       for(int k=0;k<info.m_vContacts.size();k++)
//       {
//         vContacts.push_back(info.m_vContacts[k]);
//       }
//     }
//   }
// 
//   std::cout<<"Number edges in graph: "<<nEdges<<std::endl;
//   std::cout<<"Number edges in Set: "<<mySet.size()<<std::endl;

#ifndef FEATFLOWLIB

  timer0.Start();
  PostContactAnalysis();
  dTimePostContactAnalysis+=timer0.GetTime();  
  
  //PenetrationCorrection();  

  std::cout<<"Time broadphase: "<<dTimeBroad<<std::endl;
  std::cout<<"Broadphase: number of close proximities: "<<m_BroadPhasePairs.size()<<std::endl;
  std::cout<<"Time middlephase: "<<dTimeBroad<<std::endl;  
  //printf("Number of potential collisions: %i\n",m_BroadPhasePairs.size());
  std::cout<<"Number of potential collisions: "<<m_pGraph->m_pEdges->m_vUsedCells.size()<<std::endl;
  if(nContactPoints >0)
  {
    std::cout<<"Number of actual contact points: "<<nContactPoints<<std::endl;
  }
  std::cout<<"Time narrow phase: "<<dTimeNarrow<<std::endl;

  std::cout<<"Time lcp solver total: "<<dTimeLCP<<std::endl;
  std::cout<<"Time lcp solver assembly: "<<this->m_Response->dTimeAssembly<<std::endl;
  std::cout<<"Time lcp solver: "<<this->m_Response->dTimeSolver<<std::endl;
  std::cout<<"Time lcp solver post: "<<this->m_Response->dTimeSolverPost<<std::endl;
  std::cout<<"Number of lcp solver iterations: "<<this->m_Response->GetNumIterations()<<std::endl;  
  std::cout<<"Time post-contact analysis: "<<dTimePostContactAnalysis<<std::endl;  
#endif
  m_CollInfo.clear();

  IntegrateDynamics();

  UpdateDataStructures();
}

void CCollisionPipeline::StartBroadPhase()
{
  m_BroadPhase->Start();
}

void CCollisionPipeline::StartMiddlePhase()
{

  std::set<CBroadPhasePair,Comp>::iterator liter;
  //check for every broad phase result if a corresponding edge is in the contact graph
  for(liter=m_BroadPhasePairs.begin();liter!=m_BroadPhasePairs.end();liter++)
  {
    const CBroadPhasePair &pair = *liter;
    //std::cout<<"edge: ("<<pair.m_pBody0->m_iID<<","<<pair.m_pBody1->m_iID<<")"<<std::endl;      
    CCollisionInfo *pInfo=m_pGraph->m_pEdges->Find(pair.m_pBody0->m_iID,pair.m_pBody1->m_iID);
    if(pInfo)
    {
      if(pInfo->m_iState == CCollisionInfo::CLOSEPROXIMITY)
      {
        //save the old state
        pInfo->m_iPrevState = CCollisionInfo::CLOSEPROXIMITY;
        pInfo->m_iPrevTimeStamp = pInfo->m_iTimeStamp;

        //update the state
        pInfo->m_iState = CCollisionInfo::PERSISTENT_CLOSEPROXIMITY;
        pInfo->m_iTimeStamp = m_pWorld->m_pTimeControl->GetTimeStep();

        //closeproximities can become touching contacts
        //these have to be checked by the narrow phase
      }
      else if(pInfo->m_iState == CCollisionInfo::PERSISTENT_CLOSEPROXIMITY)
      {
        //save the old state
        pInfo->m_iPrevState = CCollisionInfo::PERSISTENT_CLOSEPROXIMITY;
        pInfo->m_iPrevTimeStamp = pInfo->m_iTimeStamp;

        //update the state
        pInfo->m_iState = CCollisionInfo::PERSISTENT_CLOSEPROXIMITY;
        pInfo->m_iTimeStamp = m_pWorld->m_pTimeControl->GetTimeStep();

        //persistent closeproximities can become touching contacts
        //these have to be checked by the narrow phase
      }
      else if(pInfo->m_iState == CCollisionInfo::TOUCHING)
      {
        //save the old state
        pInfo->m_iPrevState = CCollisionInfo::TOUCHING;
        pInfo->m_iPrevTimeStamp = pInfo->m_iTimeStamp;

        //update the state
        pInfo->m_iState = CCollisionInfo::PERSISTENT_TOUCHING;
        pInfo->m_iTimeStamp = m_pWorld->m_pTimeControl->GetTimeStep();

        //touching_contacts can be resting contacts
        //or sliding contacts
        //these have to be checked by the narrow phase
      }
      else if(pInfo->m_iState == CCollisionInfo::PERSISTENT_TOUCHING)
      {
        //save the old state
        pInfo->m_iPrevState = CCollisionInfo::PERSISTENT_TOUCHING;
        pInfo->m_iPrevTimeStamp = pInfo->m_iTimeStamp;

        //update the state
        pInfo->m_iState = CCollisionInfo::PERSISTENT_TOUCHING;
        pInfo->m_iTimeStamp = m_pWorld->m_pTimeControl->GetTimeStep();
       
        //PERSISTENT_TOUCHING can be resting contacts
        //or sliding contacts
        //these have to be checked by the narrow phase
      }

      //closeproximities should not have contact points
      //the edge is already in the contact graph
      //for(int j=0;j<pInfo->m_vContacts.size();j++)
      //{
      //  //analyse the contact points

      //  //std::cout<<"found: "<<pInfo->m_vContacts[j].m_iState<<std::endl;
      //  pInfo->m_iState = CCollisionInfo::PERSISTENT_CLOSEPROXIMITY;
      //  //std::cout<<"found: "<<pInfo->m_iState<<std::endl;
      //  //std::cout<<"update state: persistent close proximity "<<std::endl;        
      //}
    }
    else
    {
      //create a new edge to add to the contact graph
      CCollisionInfo info(pair.m_pBody0,pair.m_pBody1,pair.m_pBody0->m_iID,pair.m_pBody1->m_iID);

      //set the type of the collision info
      info.m_iState = CCollisionInfo::CLOSEPROXIMITY;

      //set the timestamp
      info.m_iTimeStamp = m_pWorld->m_pTimeControl->GetTimeStep();

      //set the creation time 
      info.m_iCreationTime = m_pWorld->m_pTimeControl->GetTimeStep();
      
      //add to the graph
      m_pGraph->m_pEdges->Insert(info);

      //closeproximities can become touching contacts
      //these have to be checked by the narrow phase
    }
  }

  //now determine if there are contacts that
  //should be removed from the graph
  CCollisionHash::iterator hiter = m_pGraph->m_pEdges->begin();
  for(;hiter!=m_pGraph->m_pEdges->end();hiter++)
  {
    CCollisionInfo &info = *hiter;
    CBroadPhasePair pair(info.m_pBody1,info.m_pBody2);
    std::set<CBroadPhasePair,Comp>::iterator j = m_BroadPhasePairs.find(pair);
    if(j==m_BroadPhasePairs.end())
    {
      //if the contact is not in the broad phase,
      //it can be safely removed
      if(info.m_iState == CCollisionInfo::CLOSEPROXIMITY || info.m_iState == CCollisionInfo::PERSISTENT_CLOSEPROXIMITY)
      {
        //schedule for removal
        info.m_iState = CCollisionInfo::OBSOLETE;
      }
      else if(info.m_iState == CCollisionInfo::TOUCHING || info.m_iState == CCollisionInfo::PERSISTENT_TOUCHING)
        info.m_iState = CCollisionInfo::VANISHED_TOUCHING;
      else
      {
        //schedule for removal
        info.m_iState = CCollisionInfo::OBSOLETE;
      }
    }//end if

  }//end for
  
}

// void CCollisionPipelineRigid::StartNarrowPhase()
// {
//   int i,j;
//   //Check every pair
//   std::list<CCollisionInfo>::iterator liter;  
//   CColliderFactory colliderFactory;
// 
//   for(liter=m_CollInfo.begin();liter!=m_CollInfo.end();liter++)
//   {
// 
//     CCollisionInfo &collinfo = *liter;
//     
//     std::vector<CContact>::iterator vIter;
//     //add the CCollisionInfo here avoid, copying
// 
//     CRigidBody *p0 = collinfo.m_pBody1;
//     CRigidBody *p1 = collinfo.m_pBody2;
//     collinfo.iID1  = p0->m_iID;
//     collinfo.iID2  = p1->m_iID;    
// 
//     //get a collider
//     CCollider *collider = colliderFactory.ProduceCollider(p0,p1);
//     
//     //compute the potential contact points
//     collider->Collide(collinfo.m_vContacts,m_pTimeControl->GetDeltaT());
//     
//     //if there are contacts
//     if(!collinfo.m_vContacts.empty())
//     {
//       collinfo.m_iState       = CCollisionInfo::TOUCHING;
//       collinfo.m_dDeltaT      = m_pTimeControl->GetDeltaT();
//       collinfo.m_iNumContacts = collinfo.m_vContacts.size();
//     }
//     
//     delete collider;
//     
//   }//end for
// 
//   //DEBUGMODE: Visualize the contact points
//   std::list<CCollisionInfo>::iterator Iter;
//   std::vector<CContact>::iterator cIter;
//   //loop to determine the total number of contact points
//   for(Iter=m_CollInfo.begin();Iter!=m_CollInfo.end();Iter++)
//   {
//     CCollisionInfo &info = *Iter;
//     //printf("Pair :  (%i,%i) \n",info.m_pBody1->m_iID,info.m_pBody2->m_iID);
//     for(cIter=info.m_vContacts.begin();cIter!=info.m_vContacts.end();cIter++)
//     {
//       vContacts.push_back(*cIter);
//     }
//   }
// }

void CCollisionPipeline::UpdateContacts(CCollisionInfo &collinfo)
{
  for(int i=0;i<collinfo.m_vContacts.size();i++)
  {
    CContact &contact = collinfo.m_vContacts[i];
    VECTOR3 angVel0 = contact.m_pBody0->GetAngVel();
    VECTOR3 angVel1 = contact.m_pBody1->GetAngVel();

    //get the world-transformed inertia tensor
    MATRIX3X3 mInvInertiaTensor0 = contact.m_pBody0->GetWorldTransformedInvTensor();
    MATRIX3X3 mInvInertiaTensor1 = contact.m_pBody1->GetWorldTransformedInvTensor();
    VECTOR3 vR0 = contact.m_vPosition0-contact.m_pBody0->m_vCOM;
    VECTOR3 vR1 = contact.m_vPosition1-contact.m_pBody1->m_vCOM;

    VECTOR3 relativeVelocity = 
      (contact.m_pBody0->m_vVelocity + (VECTOR3::Cross(angVel0,vR0))
      - contact.m_pBody1->m_vVelocity - (VECTOR3::Cross(angVel1,vR1)));

    Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);

    if(relativeNormalVelocity < 0.00001)
    {
      //printf("Pre-contact normal velocity update: %lf contact enabled\n",relativeNormalVelocity);
      contact.m_iState = CCollisionInfo::TOUCHING;
    }
    else
    {
      //printf("Pre-contact normal velocity update: %lf disabling contact\n",relativeNormalVelocity);
      contact.m_iState = CCollisionInfo::TOUCHING;
    }

  }
}

void CCollisionPipeline::StartNarrowPhase()
{
  int i,j;
  //Check every pair
  CColliderFactory colliderFactory;

  CCollisionHash::iterator hiter = m_pGraph->m_pEdges->begin();
  for(;hiter!=m_pGraph->m_pEdges->end();hiter++)
  {

    CCollisionInfo &collinfo = *hiter;

    //early out
    if(collinfo.m_iState == CCollisionInfo::OBSOLETE)
      continue;

    //get pointers to the rigid bodies
    CRigidBody *p0 = collinfo.m_pBody1; //p0->m_iID==12 && p1->m_iID==14
    CRigidBody *p1 = collinfo.m_pBody2;
    
    //TODO: implement an cache contact narrow phase
    collinfo.m_vContacts.clear();

    //get a collider
    CCollider *collider = colliderFactory.ProduceCollider(p0,p1);

    //compute the potential contact points
    collider->Collide(collinfo.m_vContacts,m_pTimeControl->GetDeltaT());

    //attach the world object
    collider->SetWorld(m_pWorld);

    //if there are contacts
    if(!collinfo.m_vContacts.empty())
    {
      //closeproximity contact and persistent close proximities that become
      //touching contacts are updated
      if(collinfo.m_iState == CCollisionInfo::CLOSEPROXIMITY)
      {
        //update the state
        collinfo.m_iState = CCollisionInfo::TOUCHING;
      }
      else if(collinfo.m_iState == CCollisionInfo::PERSISTENT_CLOSEPROXIMITY)
      {
        //update the state
        collinfo.m_iState = CCollisionInfo::TOUCHING;
      }
      collinfo.m_dDeltaT      = m_pTimeControl->GetDeltaT();
      collinfo.m_iNumContacts = collinfo.m_vContacts.size();
    }
    delete collider;
  }
}

void CCollisionPipeline::UpdateDataStructures()
{

  std::vector<CRigidBody*> &vRigidBodies = m_pWorld->m_vRigidBodies;
  std::vector<CRigidBody*>::iterator rIter;

  for(rIter=vRigidBodies.begin();rIter!=vRigidBodies.end();rIter++)
  {
    CRigidBody *body = *rIter;
    //if the body has a bvh, update the bvh
    if(body->m_iShape == CRigidBody::MESH)
    {
      if(!body->IsAffectedByGravity())
        continue;
      //update bvh
      CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->m_pShape);
      C3DModel model_out(pMeshObject->m_Model);
      model_out.m_vMeshes[0].m_matTransform = body->GetTransformationMatrix();
      model_out.m_vMeshes[0].m_vOrigin = body->m_vCOM;
      model_out.m_vMeshes[0].TransformModelWorld();
      model_out.GenerateBoundingBox();
      model_out.m_vMeshes[0].GenerateBoundingBox();
      std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
      CSubDivRessources myRessources(1,6,0,model_out.GetBox(),&pTriangles);
      CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
      //update strategy is rebuilt
      pMeshObject->m_BVH.DestroyAndRebuilt(&subdivider);
    }
  }//end for
}


void CCollisionPipeline::PostContactAnalysis()
{
  
  m_pGroups.clear();
  
  m_pGraph->Update();

  if(m_pWorld->m_bExtGraph)
  {

    //assign the rigid body ids
    for(int j=0;j<m_pWorld->m_vRigidBodies.size();j++)
    {
      m_pWorld->m_vRigidBodies[j]->m_iGroup   = 0;
      m_pWorld->m_vRigidBodies[j]->m_iHeight  = 0;
      m_pWorld->m_vRigidBodies[j]->m_bVisited = false;
    }

    m_pGraph->ContactGroups(m_pGroups);
  
    std::vector<CContactGroup>::iterator i = m_pGroups.begin();
    for(;i!=m_pGroups.end();i++)
    {
      CContactGroup &group = *i;
      m_pGraph->ComputeStackLayers(group);
    
    }
  
    //ComputeStackLayers(i3d::CContactGroup& group)

  }//end if m_bExtGraph
  
}


void CCollisionPipeline::StartCollisionWall(void)
{

}

void CCollisionPipeline::StartCollisionResponseWall(void)
{
}

void CCollisionPipeline::IntegrateDynamics()
{
	this->m_pIntegrator->UpdatePosition();
}

void CCollisionPipeline::PenetrationCorrection()
{

}

void CCollisionPipeline::SolveRestingContact()
{
  m_Response->SolveRestingContact();
}

}
