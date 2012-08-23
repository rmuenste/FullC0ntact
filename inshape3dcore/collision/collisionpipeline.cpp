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
#include <collresponsesi.h>
#include <collresponse.h>
#include <world.h>
#include <iostream>
#include <timecontrol.h>
#include <colliderfactory.h>
#include <collresponselcp.h>
#include <perftimer.h>
#include <simplespatialhash.h>
#include <hspatialhash.h>
#include <meshobject.h>
#include <subdivisioncreator.h>
#include <set>
#include <collresponsesi.h>
#include <broadphasestrategyrmt.h>
#include <colliderspheresubdomain.h>
#ifdef FC_MPI_SUPPORT
#include <mpi.h>
#endif

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

void CCollisionPipeline::SetBroadPhaseNaive()
{
  m_Strategy = new CBroadPhaseStrategy(m_pWorld);
  m_Strategy->SetEPS(m_dCollEps);
  m_BroadPhase = new CBroadPhase(m_pWorld,m_Strategy);
  m_BroadPhase->m_BroadPhasePairs = &m_BroadPhasePairs;
  m_BroadPhase->m_pStrat->m_BroadPhasePairs = &m_BroadPhasePairs;  
}

void CCollisionPipeline::SetBroadPhaseSpatialHash()
{
  m_Strategy = new CBroadPhaseStrategyGrid(m_pWorld);
  m_Strategy->SetEPS(m_dCollEps);
  m_BroadPhase = new CBroadPhase(m_pWorld,m_Strategy);
  m_BroadPhase->m_BroadPhasePairs = &m_BroadPhasePairs;
  m_BroadPhase->m_pStrat->m_BroadPhasePairs = &m_BroadPhasePairs;  
  m_BroadPhase->m_pStrat->m_pImplicitGrid = new CImplicitGrid(new CSimpleSpatialHash(5001,0.05,m_pWorld->m_pBoundary->m_Values),0.05);
}

void CCollisionPipeline::SetBroadPhaseHSpatialHash()
{
  m_Strategy = new CBroadPhaseStrategyHGrid(m_pWorld);
  m_Strategy->SetEPS(m_dCollEps);
  m_BroadPhase = new CBroadPhase(m_pWorld,m_Strategy);
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

void CCollisionPipeline::Init(CWorld *pWorld, int solverType, int lcpIterations, int pipelineIterations)
{

  //set the world pointer
  m_pWorld = pWorld;
  m_pTimeControl = pWorld->m_pTimeControl;

	switch(solverType)
	{
	case 0 :
    m_Response = new CCollResponseLcp(&m_CollInfo,m_pWorld);
    m_Response->SetEPS(m_dCollEps);
    {
    CCollResponseLcp *pResponse = dynamic_cast<CCollResponseLcp *>(m_Response);
    pResponse->InitSolverPGS(lcpIterations,1.0);    
    m_iSolverType = 0;
    }
		break;
  case 1 :
    m_Response = new CCollResponseLcp(&m_CollInfo,m_pWorld);
    m_Response->SetEPS(m_dCollEps);
    {
    CCollResponseLcp *pResponse = dynamic_cast<CCollResponseLcp *>(m_Response);
    pResponse->InitSolverPGS(lcpIterations,1.0);    
    m_iSolverType = 1;
    }
    break;
  case 2 :
    m_Response = new CCollResponseSI(&m_CollInfo,m_pWorld);
    m_Response->SetEPS(m_dCollEps);
    m_iSolverType = 2;
    break;
  case 3 :
    m_Response = new CCollResponseLcp(&m_CollInfo,m_pWorld);
    m_Response->SetEPS(m_dCollEps);
    {
    CCollResponseLcp *pResponse = dynamic_cast<CCollResponseLcp *>(m_Response);
    pResponse->InitSolverPGS(lcpIterations,1.0);    
    }
    break;
  default:
    std::cerr<<"wrong solver type in: collisionpipeline.cpp"<<std::endl;
    exit(0);
    break;
	}

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

void CCollisionPipeline::SolveContactProblem()
{
  m_Response->Solve();
}

void CCollisionPipeline::StartPipeline()
{
  vContacts.clear();
  CPerfTimer timer0;
  //#ifdef  PERFTIMINGS
  double dTimeBroad=0.0;
  double dTimeMiddle=0.0;  
  double dTimeNarrow=0.0;
  double dTimeSolver=0.0;
  double dTimeLCPResting=0.0;
  double dTimePostContactAnalysis=0.0;  

#ifdef FC_MPI_SUPPORT  
  ProcessRemoteBodies();
#endif
  
  //start the broad phase collision detection
  timer0.Start();  
  StartBroadPhase();
  dTimeBroad+=timer0.GetTime();  

  //examine the broad phase results in the middle phase
  timer0.Start();    
  StartMiddlePhase();
  dTimeMiddle+=timer0.GetTime();    
  
  //start the narrow phase collision detection
  //and contact point determination
  timer0.Start();  
  StartNarrowPhase();
  dTimeNarrow+=timer0.GetTime();
  
  //remote body update phase
  
  //get timings
  timer0.Start();
  SolveContactProblem();
  
  //get timings
  dTimeSolver+=timer0.GetTime();
  //UpdateContactGraph();
  m_CollInfo.clear();

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
  
//std::set<CBroadPhasePair,Comp>::iterator liter;
//check for every broad phase result if a corresponding edge is in the contact graph
//for(liter=m_BroadPhasePairs.begin();liter!=m_BroadPhasePairs.end();liter++)
//{
//  const CBroadPhasePair &pair = *liter;
//  std::cout<<"edge: ("<<pair.m_pBody0->m_iID<<","<<pair.m_pBody1->m_iID<<")"<<std::endl;      
//}

 timer0.Start();
 PostContactAnalysis();
 dTimePostContactAnalysis+=timer0.GetTime();  

#ifndef FEATFLOWLIB
  //PenetrationCorrection();  
#ifdef FC_MPI_SUPPORT
 if(m_pWorld->m_myParInfo.GetID()==0)
 {
#endif
  std::cout<<"Time broadphase: "<<dTimeBroad<<std::endl;
  std::cout<<"Broadphase: number of close proximities: "<<m_BroadPhasePairs.size()<<std::endl;
  std::cout<<"Time middlephase: "<<dTimeMiddle<<std::endl;  

  std::cout<<"Number of potential collisions: "<<m_pGraph->m_pEdges->m_vUsedCells.size()<<std::endl;

  std::cout<<"Time narrow phase: "<<dTimeNarrow<<std::endl;

  if(m_iSolverType == 0 || m_iSolverType == 1)
  {
    std::cout<<"Number of actual contact points: "<<m_Response->m_iContactPoints<<std::endl;
    std::cout<<"Time lcp solver total: "<<dTimeSolver<<std::endl;
    std::cout<<"Time lcp solver assembly dry run: "<<this->m_Response->dTimeAssemblyDry<<std::endl;
    std::cout<<"Time lcp solver assembly: "<<this->m_Response->dTimeAssembly<<std::endl;
    std::cout<<"Time lcp solver: "<<this->m_Response->dTimeSolver<<std::endl;
    std::cout<<"Time lcp solver post: "<<this->m_Response->dTimeSolverPost<<std::endl;
    std::cout<<"Number of lcp solver iterations: "<<this->m_Response->GetNumIterations()<<std::endl;
  }
  else if(m_iSolverType == 2)
  {
    std::cout<<"Number of actual contact points: "<<m_Response->m_iContactPoints<<std::endl;
    std::cout<<"Time precomputation: "<<this->m_Response->dTimeAssemblyDry<<std::endl;
    std::cout<<"Time solver: "<<this->m_Response->dTimeSolver<<std::endl;
    std::cout<<"Time sequential impulses solver total: "<<dTimeSolver<<std::endl;
  }
  else
  {
  }

  std::cout<<"Time post-contact analysis: "<<dTimePostContactAnalysis<<std::endl;  

#ifdef FC_MPI_SUPPORT
 }
#endif
#endif
  m_CollInfo.clear();

  IntegrateDynamics();

  UpdateDataStructures();
}

void CCollisionPipeline::StartBroadPhase()
{
  //remoteBodyDetection
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
    }
    else
    {
      //create a new edge to add to the contact graph
      CCollisionInfo info(m_pWorld->m_vRigidBodies[pair.m_pBody0->m_iID],
                          m_pWorld->m_vRigidBodies[pair.m_pBody1->m_iID],
                          pair.m_pBody0->m_iID,
                          pair.m_pBody1->m_iID);

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
    CBroadPhasePair pair(info.m_pBody0,info.m_pBody1);
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
      {
        info.m_iState = CCollisionInfo::VANISHED_TOUCHING;
      }
      else
      {
        //schedule for removal
        info.m_iState = CCollisionInfo::OBSOLETE;
      }
    }//end if
  }//end for  
}

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

  CColliderFactory colliderFactory;

  CCollisionHash::iterator hiter = m_pGraph->m_pEdges->begin();
  for(;hiter!=m_pGraph->m_pEdges->end();hiter++)
  {

    CCollisionInfo &collinfo = *hiter;

    //early out
    if(collinfo.m_iState == CCollisionInfo::OBSOLETE)
      continue;

    //get pointers to the rigid bodies
    CRigidBody *p0 = collinfo.m_pBody0; //p0->m_iID==12 && p1->m_iID==14
    CRigidBody *p1 = collinfo.m_pBody1;
    
    //TODO: implement an contact cache narrow phase
    collinfo.CacheContacts();
    collinfo.m_vContacts.clear();
 
    //get a collider
    CCollider *collider = colliderFactory.ProduceCollider(p0,p1);

    //attach the world object
    collider->SetWorld(m_pWorld);

    //compute the potential contact points
    collider->Collide(collinfo.m_vContacts);

    collinfo.CheckCache();

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

void CCollisionPipeline::ProcessRemoteBodies()
{

  CBroadPhase *pBroadRemoteDetection;

  CBroadPhaseStrategy *pStrategyRemote;

  pStrategyRemote = new CBroadPhaseStrategyRmt(m_pWorld);

  pStrategyRemote->SetEPS(m_dCollEps);

  std::set<CBroadPhasePair,Comp> BroadPhasePairs;

  pBroadRemoteDetection = new CBroadPhase(m_pWorld,pStrategyRemote);

  pBroadRemoteDetection->m_BroadPhasePairs = &BroadPhasePairs;

  pBroadRemoteDetection->m_pStrat->m_BroadPhasePairs = &BroadPhasePairs;  

  pBroadRemoteDetection->m_pStrat->m_pImplicitGrid = new CImplicitGrid(new CHSpatialHash(5001,m_pWorld->m_pBoundary->m_Values,m_pWorld->m_vRigidBodies),0.05);

  pBroadRemoteDetection->Start();

  CColliderFactory colliderFactory;

  //check for new collisions with the boundary
  std::set<CBroadPhasePair,Comp>::iterator liter;
  //check for every broad phase result if a corresponding edge is in the contact graph
  for(liter=BroadPhasePairs.begin();liter!=BroadPhasePairs.end();liter++)
  {
    const CBroadPhasePair &pair = *liter;

    //If the body is local, then we have to take care
    //of the body's state. For a remote body, the local domain of
    //the remote body will update the body's state
    //CRigidBody *body = pair.GetPhysicalBody();
    CRigidBody *body0;
    CRigidBody *body1;    
    if(pair.m_pBody0->m_iShape == CRigidBody::PLANE)
    {
      body0 = pair.m_pBody1;    
      body1 = m_pWorld->m_vRigidBodies[pair.m_pBody0->m_iID];      
    }
    else
    {
      body0 = pair.m_pBody0;          
      body1 = m_pWorld->m_vRigidBodies[pair.m_pBody1->m_iID];            
    }

    if(body0->IsLocal())
    {
      //Get a collider

      //the collider should return
      //whether the body is sufficiently near the
      //domain boundary to make it a remote body
      //and it should return the IDs of the domains
      //in that the body should be made a remote body

      //get a collider
      CCollider *collider = colliderFactory.ProduceCollider(body0,body1);

      //attach the world object
      collider->SetWorld(m_pWorld);

      //compute the potential contact points
      CColliderSphereSubdomain *collSphereSub = dynamic_cast<CColliderSphereSubdomain*>(collider);

      collSphereSub->Collide();

      std::vector<CSubdomainContact>::iterator iter = collSphereSub->m_vContacts.begin();
      for(;iter!=collSphereSub->m_vContacts.end();iter++)
      {
        CSubdomainContact &contact = *iter;
        int iDomain = contact.m_iNeighbor;
        if(!body0->IsKnownInDomain(iDomain))
        {
          std::cout<<"send body: "<<body0->m_iID<<std::endl;
          body0->AddRemoteDomain(iDomain);
          m_pWorld->m_lSendList.push_back(std::pair<int,int>(body0->m_iID,iDomain));          
        } 
        else
        {
          std::cout<<"body already known in domain "<<body0->m_iID<<std::endl;          
        }
      } 
    }
  }
  delete pStrategyRemote;
  delete pBroadRemoteDetection;

  int blockcounts[2];
  MPI_Datatype particletype,oldtypes[2];
  MPI_Aint offsets[2],extent;
  
  offsets[0]     = 0;
  oldtypes[0]    = MPI_FLOAT;
  blockcounts[0] = 38;
  MPI_Type_extent(MPI_FLOAT, &extent);
  offsets[1]     = 38 * extent;
  oldtypes[1]    = MPI_INT;
  blockcounts[1] = 3;
  MPI_Type_struct(2,blockcounts,offsets,oldtypes,&particletype);
  MPI_Type_commit(&particletype);
  
  
  //set up the communication
  for(int i=0;i<m_pWorld->m_pSubBoundary->GetNumNeighbors();i++)
  {
    int iNeigh  = m_pWorld->m_pSubBoundary->GetNeighbor(i);
    int nBodies = m_pWorld->m_lSendList.size();
    int nRemote = 0;

    if(m_pWorld->m_myParInfo.GetID() == 0)
    {
      MPI_Send(&nBodies,1,MPI_INT,1,0,MPI_COMM_WORLD);
      if(nBodies > 0)
      {
        Particle *p = new Particle[nBodies];
        int *particleIDs = new int[nBodies];      
        std::list< std::pair<int,int> >::iterator j = m_pWorld->m_lSendList.begin();
        for(int l=0;j!=m_pWorld->m_lSendList.end();j++,l++)
        {
          std::pair<int,int> &myentry = *j;
          int bodyid = myentry.first;
          CRigidBody *body = m_pWorld->m_vRigidBodies[bodyid];
          p[l].x = body->m_vCOM.x;
          p[l].y = body->m_vCOM.y;
          p[l].z = body->m_vCOM.z;
          
          p[l].vx = body->m_vVelocity.x;
          p[l].vy = body->m_vVelocity.y;
          p[l].vz = body->m_vVelocity.z;
          
          p[l].ax = body->m_vAngle.x;
          p[l].ay = body->m_vAngle.y;
          p[l].az = body->m_vAngle.z;
          
          p[l].avx = body->GetAngVel().x;
          p[l].avy = body->GetAngVel().y;
          p[l].avz = body->GetAngVel().z;
          
          p[l].mx = body->m_vForce.x;
          p[l].my = body->m_vForce.y;
          p[l].mz = body->m_vForce.z;
          
          p[l].tx = body->m_vTorque.x;
          p[l].ty = body->m_vTorque.y;
          p[l].tz = body->m_vTorque.z;
          
          p[l].exx = body->m_pShape->GetAABB().m_Extends[0];
          p[l].exy = body->m_pShape->GetAABB().m_Extends[1];
          p[l].exz = body->m_pShape->GetAABB().m_Extends[2];
          
          p[l].qx = body->GetQuaternion().x;
          p[l].qy = body->GetQuaternion().y;
          p[l].qz = body->GetQuaternion().z;
          p[l].qw = body->GetQuaternion().w;
          
          p[l].a1 = body->m_InvInertiaTensor.m_d00;
          p[l].a2 = body->m_InvInertiaTensor.m_d01;
          p[l].a3 = body->m_InvInertiaTensor.m_d02;        
          
          p[l].a4 = body->m_InvInertiaTensor.m_d10;        
          p[l].a5 = body->m_InvInertiaTensor.m_d11;        
          p[l].a6 = body->m_InvInertiaTensor.m_d12;        
          
          p[l].a7 = body->m_InvInertiaTensor.m_d20;        
          p[l].a8 = body->m_InvInertiaTensor.m_d21;        
          p[l].a9 = body->m_InvInertiaTensor.m_d22;                

          p[l].density = body->m_dDensity;
          p[l].invmass = body->m_dInvMass;
          p[l].volume  = body->m_dVolume;
          p[l].restitution  = body->m_Restitution;
          
          p[l].ishape = body->m_iShape;
          
          p[l].igrav  = 1;
          p[l].origid = body->m_iID;
          
        }
        MPI_Send(p,nBodies,particletype,1,0,MPI_COMM_WORLD);
                
        //receive the remoteIDs of the bodies
        MPI_Recv(particleIDs,nBodies,MPI_INT,1,0,MPI_COMM_WORLD,MPI_STATUS_IGNORE);

        for(int k=0;k<nBodies;k++)
        {
          //save the remote ids of the bodies
          m_pWorld->m_pSubBoundary->m_iRemoteIDs[0].push_back(particleIDs[k]);
          m_pWorld->m_pSubBoundary->m_iRemoteBodies[0].push_back(p[k].origid);
          CRigidBody *remoteBody  = m_pWorld->m_vRigidBodies[p[k].origid];
          remoteBody->m_iRemoteID = particleIDs[k];
        }        
              
        //free memory for particles and ids
        delete[] particleIDs;       
        delete[] p;

      } 
    }
    else
    {
      MPI_Recv(&nRemote,1,MPI_INT,0,0,MPI_COMM_WORLD,MPI_STATUS_IGNORE);
      if(nRemote > 0)
      {
        Particle *particles = new Particle[nRemote];
        MPI_Recv(particles,nRemote,particletype,0,0,MPI_COMM_WORLD,MPI_STATUS_IGNORE);
        int *particleIDs = new int[nRemote];
        for(int k=0;k<nRemote;k++)
        {
          //create and add remote body
          CRigidBody *remoteBody = new CRigidBody(particles[k]);
          remoteBody->m_iID = m_pWorld->m_vRigidBodies.size();
          remoteBody->m_bRemote = true;
          m_pWorld->m_vRigidBodies.push_back(remoteBody);
          std::cout<<"adding remote body with id: "<<remoteBody->m_iID<<" and remote id: "<<remoteBody->m_iRemoteID<<remoteBody->m_vVelocity;
          particleIDs[k] = remoteBody->m_iID;
          m_pWorld->m_pSubBoundary->m_iRemoteIDs[0].push_back(remoteBody->m_iRemoteID);
          m_pWorld->m_pSubBoundary->m_iRemoteBodies[0].push_back(remoteBody->m_iID);                    
        }
        //send the IDs to the source domain
        MPI_Send(particleIDs,nRemote,MPI_INT,0,0,MPI_COMM_WORLD);        
        
        //free memory for particles and ids
        delete[] particleIDs;
        delete[] particles;
      }
    }

  }
  m_pWorld->m_lSendList.clear(); 
}


}
