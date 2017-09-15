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

#ifdef FC_MPI_SUPPORT
#include <mpi.h>
#endif


  template<int executionModel>
CollisionPipeline<executionModel>::CollisionPipeline()
{
  world_      = nullptr;
  strategy_   = nullptr;
  response_   = nullptr;
  broadPhase_ = nullptr;
  pipelineIterations_ = 5;
  graph_ = new ContactGraph();
  graph_->edges_ = new CollisionHash(1001);
}

  template<int executionModel>
void CollisionPipeline<executionModel>::setBroadPhaseNaive()
{
  strategy_ = new BroadPhaseStrategy(world_);
  strategy_->setEps_(collEps_);
  broadPhase_ = new BroadPhase(world_,strategy_);
  broadPhase_->broadPhasePairs_ = &broadPhasePairs_;
  broadPhase_->strategy_->broadPhasePairs_ = &broadPhasePairs_;  
}

  template<int executionModel>
void CollisionPipeline<executionModel>::setBroadPhaseSpatialHash()
{
  strategy_ = new UniformGridStrategy(world_);
  strategy_->setEps_(collEps_);
  broadPhase_ = new BroadPhase(world_,strategy_);
  broadPhase_->broadPhasePairs_ = &broadPhasePairs_;
  broadPhase_->strategy_->broadPhasePairs_ = &broadPhasePairs_;  
  broadPhase_->strategy_->implicitGrid_ = new SimpleSpatialHash(5001,0.05,world_->boundary_->extents_);
}

  template<int executionModel>
void CollisionPipeline<executionModel>::setBroadPhaseHSpatialHash()
{
  strategy_ = new HierarchicalGridStrategy(world_);
  strategy_->setEps_(collEps_);
  broadPhase_ = new BroadPhase(world_,strategy_);
  broadPhase_->broadPhasePairs_ = &broadPhasePairs_;
  broadPhase_->strategy_->broadPhasePairs_ = &broadPhasePairs_;  
  broadPhase_->strategy_->implicitGrid_ = new SpatialHashHierarchy(5001,world_->boundary_->extents_,world_->rigidBodies_);
}

  template<int executionModel>
void CollisionPipeline<executionModel>::init(World *world, int solverType, int lcpIterations, int pipelineIterations)
{

  //set the world pointer
  world_ = world;
  timeControl_ = world->timeControl_;

  switch(solverType)
  {
    case 0 :
      response_ = new CollResponseLcp(&collInfo_,world_);
      response_->SetEPS(collEps_);
      {
        CollResponseLcp *response = dynamic_cast<CollResponseLcp *>(response_);
        response->InitSolverPGS(lcpIterations,1.0);    
        solverType_ = 1;
      }
      break;
    case 1 :
      response_ = new CollResponseLcp(&collInfo_,world_);
      response_->SetEPS(collEps_);
      {
        CollResponseLcp *response = dynamic_cast<CollResponseLcp *>(response_);
        response->InitSolverPGS(lcpIterations,1.0);    
        solverType_ = 1;
      }
      break;
    case 2 :
      response_ = new CollResponseSI(&collInfo_,world_);
      response_->SetEPS(collEps_);
      solverType_ = 2;
      response_->setMaxIterations(lcpIterations);
      response_->setDefEps(5.0e-9);
      break;
    case 3 :
      response_ = new CollResponseLcp(&collInfo_,world_);
      response_->SetEPS(collEps_);
      {
        CollResponseLcp *pResponse = dynamic_cast<CollResponseLcp *>(response_);
        pResponse->InitSolverPGS(lcpIterations,1.0);    
      }
      break;
    default:
      std::cerr<<"wrong solver type in: collisionpipeline.cpp"<<std::endl;
      std::exit(EXIT_FAILURE);
      break;
  }
  world_->graph_ = graph_;
  pipelineIterations_ = pipelineIterations;

}

  template<int executionModel>
CollisionPipeline<executionModel>::CollisionPipeline(const CollisionPipeline &copy)
{
  broadPhase_ = copy.broadPhase_;
  strategy_   = copy.strategy_;
  world_      = copy.world_;
  collInfo_   = copy.collInfo_;
}

  template<int executionModel>
CollisionPipeline<executionModel>::~CollisionPipeline()
{
  if(strategy_ != nullptr)
  {
    delete strategy_;
    strategy_ = nullptr;
  }
  if(response_ != nullptr)
  {
    delete response_;
    response_ = nullptr;
  }
  if(broadPhase_ != nullptr)
  {
    delete broadPhase_;
    broadPhase_ = nullptr;
  }
  delete graph_;   
}

  template<int executionModel>
void CollisionPipeline<executionModel>::solveContactProblem()
{
  response_->Solve();
}

  template<int executionModel>
void CollisionPipeline<executionModel>::startPipeline()
{
  contacts_.clear();
  CPerfTimer timer0;
  //#ifdef  PERFTIMINGS
  double timeBroad=0.0;
  double timeMiddle=0.0;  
  double timeNarrow=0.0;
  double timeSolver=0.0;
  double timeLCPResting=0.0;
  double timePostContactAnalysis=0.0;  

  updateDataStructures();

#ifdef FC_MPI_SUPPORT  
  ProcessRemoteBodies();
#endif


  //start the broad phase collision detection

  timer0.Start();  
  startBroadPhase();
  timeBroad+=timer0.GetTime();

  //examine the broad phase results in the middle phase
  timer0.Start();    
  startMiddlePhase();
  timeMiddle+=timer0.GetTime();    

  //start the narrow phase collision detection
  //and contact point determination
  timer0.Start();  
  startNarrowPhase();
  timeNarrow+=timer0.GetTime();

  if(world_->solverType_ == 2)
  {

    for (auto &body : world_->rigidBodies_)
    {

      if(!body->isAffectedByGravity())
        continue;  

      //velocity update
      if(body->isAffectedByGravity())
      {
        body->velocity_ += world_->getGravityEffect(body) * world_->timeControl_->GetDeltaT();
      }

    }

  }

  timer0.Start();
  //get timings
  //if(world_->timeControl_->m_iTimeStep!=0)
  {
    solveContactProblem();
  }

  //get timings
  timeSolver+=timer0.GetTime();
  //UpdateContactGraph();
  collInfo_.clear();

  int contactPoints=0;
  int edges=0;
  int realEdges=0;

  //   CCollisionHash::iterator hiter = m_pGraph->m_pEdges->begin();
  //   std::set<CCollisionInfo,CompColl> mySet;
  //   if(!m_pGraph->m_pEdges->IsEmpty())
  //   {
  //     for(;hiter!=m_pGraph->m_pEdges->end();hiter++)
  //     {
  //       CCollisionInfo &info = *hiter;
  //       mySet.insert(info);
  //       edges++;
  //       contactPoints+=info.m_vContacts.size();
  //       for(int k=0;k<info.m_vContacts.size();k++)
  //       {
  //         vContacts.push_back(info.m_vContacts[k]);
  //       }
  //     }
  //   }
  // 
  //   std::cout<<"Number edges in graph: "<<edges<<std::endl;
  //   std::cout<<"Number edges in Set: "<<mySet.size()<<std::endl;

  //std::set<CBroadPhasePair,Comp>::iterator liter;
  //check for every broad phase result if a corresponding edge is in the contact graph
  //for(liter=m_BroadPhasePairs.begin();liter!=m_BroadPhasePairs.end();liter++)
  //{
  //  const CBroadPhasePair &pair = *liter;
  //  std::cout<<"edge: ("<<pair.m_pBody0->m_iID<<","<<pair.m_pBody1->m_iID<<")"<<std::endl;      
  //}

  timer0.Start();
  postContactAnalysis();
  timePostContactAnalysis+=timer0.GetTime();  

#ifndef FEATFLOWLIB
  //PenetrationCorrection();  
#ifdef FC_MPI_SUPPORT
  if(m_pWorld->m_myParInfo.GetID()==0)
  {
#endif

#ifndef FC_SILENT
    std::cout<<"Time broadphase: "<<timeBroad<<std::endl;
    std::cout<<"Broadphase: number of close proximities: "<<broadPhasePairs_.size()<<std::endl;

    //      std::set<BroadPhasePair,Comp>::iterator liter;
    //      //check for every broad phase result if a corresponding edge is in the contact graph
    //      for (liter = broadPhasePairs_.begin(); liter != broadPhasePairs_.end(); liter++)
    //      {
    //        const BroadPhasePair &pair = *liter;
    //        std::cout << "edge: (" << pair.m_pBody0->iID_ << "," << pair.m_pBody1->iID_ << ")" << std::endl;
    //      }

    std::cout<<"Time middlephase: "<<timeMiddle<<std::endl;  

    std::cout<<"Number of potential collisions: "<<graph_->edges_->usedCells_.size()<<std::endl;

    std::cout<<"Time narrow phase: "<<timeNarrow<<std::endl;

    if(solverType_ == 0 || solverType_ == 1)
    {
      std::cout<<"Number of actual contact points: "<<response_->m_iContactPoints<<std::endl;
      std::cout<<"Time lcp solver total: "<<timeSolver<<std::endl;
      std::cout<<"Time lcp solver assembly dry run: "<<this->response_->dTimeAssemblyDry<<std::endl;
      std::cout<<"Time lcp solver assembly: "<<this->response_->dTimeAssembly<<std::endl;
      std::cout<<"Time lcp solver: "<<this->response_->dTimeSolver<<std::endl;
      std::cout<<"Time lcp solver post: "<<this->response_->dTimeSolverPost<<std::endl;
      std::cout<<"Number of lcp solver iterations: "<<this->response_->GetNumIterations()<<std::endl;
    }
    else if(solverType_ == 2)
    {
      std::cout<<"Number of actual contact points: "<<response_->m_iContactPoints<<std::endl;
      std::cout<<"Time precomputation: "<<this->response_->dTimeAssemblyDry<<std::endl;
      std::cout<<"Time solver: "<<this->response_->dTimeSolver<<std::endl;
      std::cout<<"Time sequential impulses solver total: "<<timeSolver<<std::endl;
    }
    else
    {
    }

    std::cout << "total time[ms]: " << timeBroad + timeMiddle + timeNarrow + timeSolver << std::endl;

    std::cout<<"Time post-contact analysis: "<<timePostContactAnalysis<<std::endl;  
#endif

#ifdef FC_MPI_SUPPORT
  }
#endif
#endif
  collInfo_.clear();

  //===============================================

  //===============================================

  integrateDynamics();

  updateDataStructures();
}

  template<int executionModel>
void CollisionPipeline<executionModel>::startBroadPhase()
{
  //remoteBodyDetection
  broadPhase_->start();
}

  template<int executionModel>
void CollisionPipeline<executionModel>::startMiddlePhase()
{

  std::set<BroadPhasePair,Comp>::iterator liter;
  //check for every broad phase result if a corresponding edge is in the contact graph
  for(liter=broadPhasePairs_.begin();liter!=broadPhasePairs_.end();liter++)
  {
    const BroadPhasePair &pair = *liter;
    //std::cout<<"edge: ("<<pair.m_pBody0->m_iID<<","<<pair.m_pBody1->m_iID<<")"<<std::endl;      
    CollisionInfo *pInfo=graph_->edges_->find(pair.m_pBody0->iID_,pair.m_pBody1->iID_);
    if(pInfo)
    {
      if(pInfo->m_iState == CollisionInfo::CLOSEPROXIMITY)
      {
        //save the old state
        pInfo->m_iPrevState = CollisionInfo::CLOSEPROXIMITY;
        pInfo->m_iPrevTimeStamp = pInfo->m_iTimeStamp;

        //update the state
        pInfo->m_iState = CollisionInfo::PERSISTENT_CLOSEPROXIMITY;
        pInfo->m_iTimeStamp = world_->timeControl_->GetTimeStep();

        //closeproximities can become touching contacts
        //these have to be checked by the narrow phase
      }
      else if(pInfo->m_iState == CollisionInfo::PERSISTENT_CLOSEPROXIMITY)
      {
        //save the old state
        pInfo->m_iPrevState = CollisionInfo::PERSISTENT_CLOSEPROXIMITY;
        pInfo->m_iPrevTimeStamp = pInfo->m_iTimeStamp;

        //update the state
        pInfo->m_iState = CollisionInfo::PERSISTENT_CLOSEPROXIMITY;
        pInfo->m_iTimeStamp = world_->timeControl_->GetTimeStep();

        //persistent closeproximities can become touching contacts
        //these have to be checked by the narrow phase
      }
      else if(pInfo->m_iState == CollisionInfo::TOUCHING)
      {
        //save the old state
        pInfo->m_iPrevState = CollisionInfo::TOUCHING;
        pInfo->m_iPrevTimeStamp = pInfo->m_iTimeStamp;

        //update the state
        pInfo->m_iState = CollisionInfo::PERSISTENT_TOUCHING;
        pInfo->m_iTimeStamp = world_->timeControl_->GetTimeStep();

        //touching_contacts can be resting contacts
        //or sliding contacts
        //these have to be checked by the narrow phase
      }
      else if(pInfo->m_iState == CollisionInfo::PERSISTENT_TOUCHING)
      {
        //save the old state
        pInfo->m_iPrevState = CollisionInfo::PERSISTENT_TOUCHING;
        pInfo->m_iPrevTimeStamp = pInfo->m_iTimeStamp;

        //update the state
        pInfo->m_iState = CollisionInfo::PERSISTENT_TOUCHING;
        pInfo->m_iTimeStamp = world_->timeControl_->GetTimeStep();

        //PERSISTENT_TOUCHING can be resting contacts
        //or sliding contacts
        //these have to be checked by the narrow phase
      }
    }
    else
    {
      //create a new edge to add to the contact graph
      CollisionInfo info(world_->rigidBodies_[pair.m_pBody0->iID_],
          world_->rigidBodies_[pair.m_pBody1->iID_],
          pair.m_pBody0->iID_,
          pair.m_pBody1->iID_);

      //set the type of the collision info
      info.m_iState = CollisionInfo::CLOSEPROXIMITY;

      //set the timestamp
      info.m_iTimeStamp = world_->timeControl_->GetTimeStep();

      //set the creation time 
      info.m_iCreationTime = world_->timeControl_->GetTimeStep();

      //add to the graph
      graph_->edges_->insert(info);

      //closeproximities can become touching contacts
      //these have to be checked by the narrow phase
    }
  }

  //now determine if there are contacts that
  //should be removed from the graph
  CollisionHash::iterator hiter = graph_->edges_->begin();
  for(;hiter!=graph_->edges_->end();hiter++)
  {
    CollisionInfo &info = *hiter;
    BroadPhasePair pair(info.m_pBody0,info.m_pBody1);
    std::set<BroadPhasePair,Comp>::iterator j = broadPhasePairs_.find(pair);
    if(j==broadPhasePairs_.end())
    {
      //if the contact is not in the broad phase,
      //it can be safely removed
      if(info.m_iState == CollisionInfo::CLOSEPROXIMITY || info.m_iState == CollisionInfo::PERSISTENT_CLOSEPROXIMITY)
      {
        //schedule for removal
        info.m_iState = CollisionInfo::OBSOLETE;
      }
      else if(info.m_iState == CollisionInfo::TOUCHING || info.m_iState == CollisionInfo::PERSISTENT_TOUCHING)
      {
        info.m_iState = CollisionInfo::VANISHED_TOUCHING;
      }
      else
      {
        //schedule for removal
        info.m_iState = CollisionInfo::OBSOLETE;
      }
    }//end if
  }//end for  

  //graph_->update();  

}

  template<int executionModel>
void CollisionPipeline<executionModel>::updateContacts(CollisionInfo &collinfo)
{
  for(int i=0;i<collinfo.m_vContacts.size();i++)
  {
    Contact &contact = collinfo.m_vContacts[i];
    VECTOR3 angVel0 = contact.m_pBody0->getAngVel();
    VECTOR3 angVel1 = contact.m_pBody1->getAngVel();

    //get the world-transformed inertia tensor
    MATRIX3X3 mInvInertiaTensor0 = contact.m_pBody0->getWorldTransformedInvTensor();
    MATRIX3X3 mInvInertiaTensor1 = contact.m_pBody1->getWorldTransformedInvTensor();
    VECTOR3 vR0 = contact.m_vPosition0-contact.m_pBody0->com_;
    VECTOR3 vR1 = contact.m_vPosition1-contact.m_pBody1->com_;

    VECTOR3 relativeVelocity = 
      (contact.m_pBody0->velocity_ + (VECTOR3::Cross(angVel0,vR0))
       - contact.m_pBody1->velocity_ - (VECTOR3::Cross(angVel1,vR1)));

    Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);

    if(relativeNormalVelocity < 0.00001)
    {
      //printf("Pre-contact normal velocity update: %lf contact enabled\n",relativeNormalVelocity);
      contact.m_iState = CollisionInfo::TOUCHING;
    }
    else
    {
      //printf("Pre-contact normal velocity update: %lf disabling contact\n",relativeNormalVelocity);
      contact.m_iState = CollisionInfo::TOUCHING;
    }
  }
}

  template<int executionModel>
void CollisionPipeline<executionModel>::startNarrowPhase()
{

  ColliderFactory colliderFactory;

  CollisionHash::iterator hiter = graph_->edges_->begin();
  for(;hiter!=graph_->edges_->end();hiter++)
  {

    CollisionInfo &collinfo = *hiter;

    //early out
    if(collinfo.m_iState == CollisionInfo::OBSOLETE)
      continue;

    //get pointers to the rigid bodies
    RigidBody *p0 = collinfo.m_pBody0;
    RigidBody *p1 = collinfo.m_pBody1;

    //TODO: implement an contact cache narrow phase
    collinfo.CacheContacts();
    collinfo.m_vContacts.clear();

    //get a collider
    Collider *collider = colliderFactory.ProduceCollider(p0,p1);

    //attach the world object
    collider->setWorld(world_);

    //compute the potential contact points
    collider->collide(collinfo.m_vContacts);

    for (auto &c : collinfo.m_vContacts)
    {
      c.m_iCreationTime = world_->timeControl_->GetTimeStep();
      c.m_iTimeStamp = world_->timeControl_->GetTimeStep();
      c.m_iPrevTimeStamp = world_->timeControl_->GetTimeStep();
    }
    collinfo.CheckCache();

    //if there are contacts
    if(!collinfo.m_vContacts.empty())
    {
      //closeproximity contact and persistent close proximities that become
      //touching contacts are updated
      if(collinfo.m_iState == CollisionInfo::CLOSEPROXIMITY)
      {
        //update the state
        collinfo.m_iState = CollisionInfo::TOUCHING;
      }
      else if(collinfo.m_iState == CollisionInfo::PERSISTENT_CLOSEPROXIMITY)
      {
        //update the state
        collinfo.m_iState = CollisionInfo::TOUCHING;
      }
      else
      {
        //update the state
        collinfo.m_iState = CollisionInfo::TOUCHING;

      }
      collinfo.m_iNumContacts = collinfo.m_vContacts.size();
    }
    delete collider;
  }

  if (world_->extGraph_)
  {
    groups_.clear();
    //assign the rigid body ids
    for (unsigned j(0); j<world_->rigidBodies_.size(); ++j)
    {
      world_->rigidBodies_[j]->group_ = 0;
      world_->rigidBodies_[j]->height_ = 0;
      world_->rigidBodies_[j]->visited_ = false;
    }

    graph_->contactGroups(groups_);

    //for (auto &group : groups_)
    //{
    //  graph_->computeStackLayers(group);
    //}

  }//end if m_bExtGraph

}

  template<int executionModel>
void CollisionPipeline<executionModel>::updateDataStructures()
{

  for (auto const &body : world_->rigidBodies_)
  {
    //if the body has a bvh, update the bvh
    if(body->shapeId_ == RigidBody::MESH)
    {
      //update bvh
      CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->shape_);
      Model3D model_out(pMeshObject->m_Model);
      model_out.meshes_[0].transform_ = body->getTransformationMatrix();
      model_out.meshes_[0].com_ = body->com_;
      model_out.meshes_[0].TransformModelWorld();
      model_out.generateBoundingBox();
      model_out.meshes_[0].generateBoundingBox();
      std::vector<Triangle3r> pTriangles = model_out.genTriangleVector();
      CSubDivRessources myRessources(1,4,0,model_out.getBox(),&pTriangles);
      CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
      //update strategy is rebuilt
      pMeshObject->m_BVH.DestroyAndRebuilt(&subdivider);
    }
  }//end for

}

  template<int executionModel>
void CollisionPipeline<executionModel>::postContactAnalysis()
{

  groups_.clear();

  graph_->update();

  if(world_->extGraph_)
  {

    //assign the rigid body ids
    for(unsigned j(0); j<world_->rigidBodies_.size(); ++j)
    {
      world_->rigidBodies_[j]->group_   = 0;
      world_->rigidBodies_[j]->height_  = 0;
      world_->rigidBodies_[j]->visited_ = false;
    }

    graph_->contactGroups(groups_);

    //for (auto &group : groups_)
    //{
    //  graph_->computeStackLayers(group);
    //}

  }//end if m_bExtGraph

}

  template<int executionModel>
void CollisionPipeline<executionModel>::startCollisionWall(void)
{

}

  template<int executionModel>
void CollisionPipeline<executionModel>::startCollisionResponseWall(void)
{
}

  template<int executionModel>
void CollisionPipeline<executionModel>::integrateDynamics()
{
  this->integrator_->updatePosition();
}

  template<int executionModel>
void CollisionPipeline<executionModel>::penetrationCorrection()
{

}

  template<int executionModel>
void CollisionPipeline<executionModel>::processRemoteBodies()
{

  BroadPhase *pBroadRemoteDetection;

  BroadPhaseStrategy *pStrategyRemote;

  pStrategyRemote = new RemoteBodyStrategy(world_);

  pStrategyRemote->setEps_(collEps_);

  std::set<BroadPhasePair,Comp> BroadPhasePairs;

  pBroadRemoteDetection = new BroadPhase(world_,pStrategyRemote);

  pBroadRemoteDetection->broadPhasePairs_ = &BroadPhasePairs;

  pBroadRemoteDetection->strategy_->broadPhasePairs_ = &BroadPhasePairs;  

  pBroadRemoteDetection->strategy_->implicitGrid_ = new SpatialHashHierarchy(5001,world_->boundary_->extents_,world_->rigidBodies_);

  pBroadRemoteDetection->start();

  ColliderFactory colliderFactory;

  //check for new collisions with the boundary
  std::set<BroadPhasePair,Comp>::iterator liter;
  //check for every broad phase result if a corresponding edge is in the contact graph
  for(liter=BroadPhasePairs.begin();liter!=BroadPhasePairs.end();liter++)
  {
    const BroadPhasePair &pair = *liter;

    //If the body is local, then we have to take care
    //of the body's state. For a remote body, the local domain of
    //the remote body will update the body's state
    //CRigidBody *body = pair.GetPhysicalBody();
    RigidBody *body0;
    RigidBody *body1;    
    if(pair.m_pBody0->shapeId_ == RigidBody::PLANE)
    {
      body0 = pair.m_pBody1;    
      body1 = world_->rigidBodies_[pair.m_pBody0->iID_];      
    }
    else
    {
      body0 = pair.m_pBody0;          
      body1 = world_->rigidBodies_[pair.m_pBody1->iID_];            
    }

    if(body0->isLocal())
    {
      //Get a collider

      //the collider should return
      //whether the body is sufficiently near the
      //domain boundary to make it a remote body
      //and it should return the IDs of the domains
      //in that the body should be made a remote body

      //get a collider
      Collider *collider = colliderFactory.ProduceCollider(body0,body1);

      //attach the world object
      collider->setWorld(world_);

      //compute the potential contact points
      ColliderSphereSubdomain *collSphereSub = dynamic_cast<ColliderSphereSubdomain*>(collider);

      collSphereSub->Collide();

      std::vector<SubdomainContact>::iterator iter = collSphereSub->m_vContacts.begin();
      for(;iter!=collSphereSub->m_vContacts.end();iter++)
      {
        SubdomainContact &contact = *iter;
        int iDomain = contact.m_iNeighbor;
        if(!body0->isKnownInDomain(iDomain))
        {
          std::cout<<"send body: "<<body0->iID_<<std::endl;
          body0->addRemoteDomain(iDomain);
          world_->sendList_.push_back(std::pair<int,int>(body0->iID_,iDomain));          
        } 
        else
        {
          std::cout<<"body already known in domain "<<body0->iID_<<std::endl;          
        }
      } 
    }
  }
  delete pStrategyRemote;
  delete pBroadRemoteDetection;

#ifdef FC_MPI_SUPPORT 
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
#endif      
}



