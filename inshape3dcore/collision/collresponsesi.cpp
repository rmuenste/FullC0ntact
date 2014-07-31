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

#include "collresponsesi.h"
#include <3dmodel.h>
#include <3dmesh.h>
#include <rigidbody.h>
#include <world.h>
#include <vectorn.h>
#include <perftimer.h>
#ifdef FC_MPI_SUPPORT
#include <mpi.h>
#endif

namespace i3d {

/**
 * You need indeed two friction directions then. It might help that you align the first friction axis 
 * with the relative velocity in the tangent plane. It is also important to project the last friction 
 * impulse onto the new friction directions for warmstarting. 
 * Say you have your two accumulated friction impulses lambda1 and lambda2 and 
 * the associated direction vectors tangent1 and tangent2 from the last and current frame. 
 * Then you need to do the following to project the impulse:
 * Vec3 OldImpulse = Lambda1 * OldTangent1 + Lambda2 * OldTangent2;
 * Lambda1 = dot( Lambda1, NewTangent1 );
 * Lambda2 = dot( Lambda2, NewTangent2 );
 * Don't skip friction if the relative velocity in the tangent plane is zero. 
 * You create an arbitrary frame then. You can look at dPlaneSpace() in the ODE if you need an example.
*/
  
CollResponseSI::CollResponseSI(void)
{
    m_dBiasFactor = Real(0.1);
}

CollResponseSI::~CollResponseSI(void)
{

}

CollResponseSI::CollResponseSI(std::list<CollisionInfo> *CollInfo, World *pWorld) : CollResponse(CollInfo,pWorld)
{
    m_dBiasFactor = Real(0.1);
}

void CollResponseSI::Solve()
{

  //return status of our solver
  int ireturnStatus;

  //number of iterations
  int iterations;

  if(this->m_pGraph->edges_->isEmpty())
    return;

  int i,j;
  Real deltaT = m_pWorld->timeControl_->GetDeltaT();

  CPerfTimer timer0;
  dTimeAssembly = 0;
  dTimeSolver = 0;
  dTimeSolverPost = 0;
  dTimeAssemblyDry = 0;

  m_dBiasFactor = 0.0;

  //number of different contacts
  int nContacts=0;

  //in the SI framework we apply the external forces before
  //we call the constraint force solver
  std::vector<RigidBody*> &vRigidBodies = m_pWorld->rigidBodies_;
  std::vector<RigidBody*>::iterator rIter;

  for(rIter=vRigidBodies.begin();rIter!=vRigidBodies.end();rIter++)
  {

    RigidBody *body = *rIter;

    if(!body->isAffectedByGravity())
      continue;

    VECTOR3 &vel    = body->velocity_;
  }//end for
  
  
//   int count = 0;
//   for(rIter=vRigidBodies.begin();rIter!=vRigidBodies.end();rIter++)
//   {
// 
//     CRigidBody *body = *rIter;
// 
//     if(body->m_iShape == CRigidBody::BOUNDARYBOX || !body->IsAffectedByGravity())
//       continue;
// 
//     VECTOR3 &pos    = body->m_vCOM;
//     VECTOR3 &vel    = body->m_vVelocity;
//     //std::cout<<"body id/remoteid/velocity: "<<body->m_iID<<" "<<body->m_iRemoteID<<" "<<body->m_vVelocity;
//     body->SetAngVel(VECTOR3(0,0,0));
// 
//     //velocity update
//     if(body->IsAffectedByGravity())
//     {
//       vel += m_pWorld->GetGravityEffect(body) * m_pWorld->m_pTimeControl->GetDeltaT();
//     }
// 
//     //std::cout<<"body id/remoteid/velocity: "<<body->m_iID<<" "<<body->m_iRemoteID<<" "<<body->m_vVelocity;
// 
//   }//end for

  timer0.Start();
  m_iContactPoints = 0;
  CollisionHash::iterator hiter = m_pGraph->edges_->begin();
  for(;hiter!=m_pGraph->edges_->end();hiter++)
  {
    CollisionInfo &info = *hiter;
    if(!info.m_vContacts.empty())
    {
      PreComputeConstants(info);
    }
  }

  dTimeAssemblyDry+=timer0.GetTime();

  //initialize the defect vector
  defect_ = VectorNr(m_iContactPoints);
  oldDefect_ = 0.0;
  timer0.Start();
  //call the sequential impulses solver with a fixed
  //number of iterations
  for(iterations=0;iterations<50;iterations++)
  {
    
    for(rIter=vRigidBodies.begin();rIter!=vRigidBodies.end();rIter++)
    {

      RigidBody *body = *rIter;

      if(!body->isAffectedByGravity())
        continue;

      VECTOR3 &vel    = body->velocity_;
      //backup the velocity
      body->oldVel_ = body->velocity_;
      //std::cout<<"body id/remoteid/velocity: "<<body->m_iID<<" "<<body->m_iRemoteID<<" "<<body->m_vVelocity;       
    }//end for
        
    hiter = m_pGraph->edges_->begin();
    for(;hiter!=m_pGraph->edges_->end();hiter++)
    {
      CollisionInfo &info = *hiter;
      if(!info.m_vContacts.empty())
      {
#ifdef FC_DEBUG        
        if(info.m_pBody0->isAffectedByGravity() && info.m_pBody1->isAffectedByGravity())
        {
          std::cout << "Pair: " << info.m_pBody0->iID_ << " " << info.m_pBody1->iID_ << std::endl;          
          std::cout<<"velocity before0: "<<info.m_pBody0->velocity_;               
          std::cout<<"velocity before1: "<<info.m_pBody1->velocity_;                       
        }
#endif        
        ApplyImpulse(info);
#ifdef FC_DEBUG                
        if(info.m_pBody0->isAffectedByGravity() && info.m_pBody1->isAffectedByGravity())
        {
          std::cout<<"velocity after0: "<<info.m_pBody0->velocity_;               
          std::cout<<"velocity after1: "<<info.m_pBody1->velocity_;                       
        }
#endif                
      }      
    }

    //Real newDefect_ = ComputeDefect();
    //oldDefect_ = newDefect_;
    //if(fabs(oldDefect_ - newDefect_) < 1.0e-6)break;
#ifdef FC_MPI_SUPPORT       
    //we now have to synchronize the remote bodies
    if(m_pWorld->m_myParInfo.GetID() == 0)      
    {
      int nBodies = m_pWorld->m_pSubBoundary->m_iRemoteIDs[0].size(); 
      //std::cout<<"Number of remotes in 0 "<<nBodies<<std::endl; 
      //send struct {diff,targetID}
      Real *diffs = new Real[3*nBodies];
      int *remotes = new int[nBodies];
      
      Real *diffs2 = new Real[3*nBodies];
      int *remotes2 = new int[nBodies];
              
      for(int k=0;k<nBodies;k++)
      {
        remotes[k]=m_pWorld->m_pSubBoundary->m_iRemoteIDs[0][k];
        CRigidBody *body = m_pWorld->m_vRigidBodies[m_pWorld->m_pSubBoundary->m_iRemoteBodies[0][k]];
        diffs[3*k]   = body->m_vVelocity.x - body->m_vOldVel.x;
        diffs[3*k+1] = body->m_vVelocity.y - body->m_vOldVel.y;
        diffs[3*k+2] = body->m_vVelocity.z - body->m_vOldVel.z;
/*        std::cout<<"myid= 0 /id/velocity update: "<<body->m_iID<<" "<<diffs[3*k+2]<<" "<<body->m_vVelocity;                
        std::cout<<VECTOR3(diffs[3*k],diffs[3*k+1],diffs[3*k+2]);        */
      }
      
      MPI_Send(remotes,nBodies,MPI_INT,1,0,MPI_COMM_WORLD);
      MPI_Send(diffs,3*nBodies,MPI_DOUBLE,1,0,MPI_COMM_WORLD);                     
      MPI_Recv(remotes2,nBodies,MPI_INT,1,0,MPI_COMM_WORLD,MPI_STATUS_IGNORE);
      MPI_Recv(diffs2,3*nBodies,MPI_DOUBLE,1,0,MPI_COMM_WORLD,MPI_STATUS_IGNORE); 
      
      //apply velocity difference    
      for(int k=0;k<nBodies;k++)
      {          
        CRigidBody *body = m_pWorld->m_vRigidBodies[remotes2[k]];
        //std::cout<<"myid= 0 /id/velocity update from 1: "<<body->m_iID<<" "<<diffs2[3*k+2]<<" "<<body->m_vVelocity;                                
        body->m_vVelocity.x += diffs2[3*k];
        body->m_vVelocity.y += diffs2[3*k+1];
        body->m_vVelocity.z += diffs2[3*k+2];
        //std::cout<<"myid= 0 synced velocity: "<<body->m_vVelocity;                       
      }
                      
      delete[] diffs;
      delete[] remotes;
      
      delete[] diffs2;
      delete[] remotes2;
    }
    else
    {
      //max_remotes = max size of int the m_iRemoteBodies vector
      //compute diffs
      //MPI_Gather(sendStruct,max_remotes, particletype, receiveBuffer, 1, root, GroupComm)
      //apply
      //MPI_Scatter()

      int nBodies = m_pWorld->m_pSubBoundary->m_iRemoteIDs[0].size();
      //std::cout<<"Number of remotes in 1 "<<nBodies<<std::endl;         
      //send struct {diff,targetID}
      Real *diffs = new Real[3*nBodies];
      int *remotes = new int[nBodies];
      
      Real *diffs2 = new Real[3*nBodies];
      int *remotes2 = new int[nBodies];
              
      for(int k=0;k<nBodies;k++)
      {
        remotes[k]=m_pWorld->m_pSubBoundary->m_iRemoteIDs[0][k];
        CRigidBody *body = m_pWorld->m_vRigidBodies[m_pWorld->m_pSubBoundary->m_iRemoteBodies[0][k]];
        diffs[3*k]   = body->m_vVelocity.x - body->m_vOldVel.x;
        diffs[3*k+1] = body->m_vVelocity.y - body->m_vOldVel.y;
        diffs[3*k+2] = body->m_vVelocity.z - body->m_vOldVel.z;
        //std::cout<<"velocity difference: "<<body->m_vVelocity - body->m_vOldVel;
        //std::cout<<"body id/remoteid/velocity: "<<body->m_iID<<" "<<body->m_iRemoteID<<" "<<body->m_vVelocity;                    
      }
      
      MPI_Recv(remotes2,nBodies,MPI_INT,0,0,MPI_COMM_WORLD,MPI_STATUS_IGNORE);
      MPI_Recv(diffs2,3*nBodies,MPI_DOUBLE,0,0,MPI_COMM_WORLD,MPI_STATUS_IGNORE);                
      MPI_Send(remotes,nBodies,MPI_INT,0,0,MPI_COMM_WORLD);
      MPI_Send(diffs,3*nBodies,MPI_DOUBLE,0,0,MPI_COMM_WORLD);                     
      
      for(int k=0;k<nBodies;k++)
      {          
        CRigidBody *body = m_pWorld->m_vRigidBodies[remotes2[k]];
        //std::cout<<"myid= 1 /id/velocity update from 0: "<<body->m_iID<<" "<<diffs2[3*k+2]<<" "<<body->m_vVelocity;
        body->m_vVelocity.x += diffs2[3*k];
        body->m_vVelocity.y += diffs2[3*k+1];
        body->m_vVelocity.z += diffs2[3*k+2];
        //std::cout<<VECTOR3(diffs2[3*k],diffs2[3*k+1],diffs2[3*k+2]);
        //std::cout<<"myid= 1 synced velocity: "<<body->m_vVelocity;
        //std::cout<<"myid= 1 /id/velocity update: "<<body->m_iID<<" "<<remotes2[k]<<" "<<diffs2[3*k+2]<<" "<<body->m_vVelocity;
        //std::cout<<"myid= 1 /body id/remoteid/velocity: "<<body->m_iID<<" "<<body->m_iRemoteID<<" "<<body->m_vVelocity;
      }
      
      for(rIter=vRigidBodies.begin();rIter!=vRigidBodies.end();rIter++)
      {

        CRigidBody *body = *rIter;

        if(body->m_iShape == CRigidBody::BOUNDARYBOX || !body->IsAffectedByGravity())
          continue;

        VECTOR3 &vel    = body->m_vVelocity;
        //backup the velocity
        body->m_vOldVel = body->m_vVelocity;
        //std::cout<<"body id/remoteid/velocity: "<<body->m_iID<<" "<<body->m_iRemoteID<<" "<<body->m_vVelocity;
      }//end for
      
              
      delete[] diffs;
      delete[] remotes;
      
      delete[] diffs2;
      delete[] remotes2;
              
    }            
#endif
    
    //std::cout<<"Iteration: "<<iterations <<" "<<ComputeDefect()<<std::endl;
  }

  dTimeSolver+=timer0.GetTime();
  
  for(rIter=vRigidBodies.begin();rIter!=vRigidBodies.end();rIter++)
  {

    RigidBody *body = *rIter;

    if(!body->isAffectedByGravity())
      continue;

    VECTOR3 &vel = body->velocity_;

  }//end for
      
}//end Solve

void CollResponseSI::PreComputeConstants(CollisionInfo &ContactInfo)
{
  Real penEps = 0.0006;
  std::vector<Contact>::iterator iter;
	for(iter=ContactInfo.m_vContacts.begin();iter!=ContactInfo.m_vContacts.end();iter++)
	{
      
    Contact &contact = *iter;

    if(contact.m_iState != CollisionInfo::TOUCHING)
      continue;

    m_iContactPoints++;

    ComputeTangentSpace(contact.m_vNormal,contact.m_vTangentU,contact.m_vTangentV);
    
    //the massinverse is needed
    Real massinv = contact.m_pBody0->invMass_ + contact.m_pBody1->invMass_;

    VECTOR3 vR0  = contact.m_vPosition0 - contact.m_pBody0->com_;
    VECTOR3 vR1  = contact.m_vPosition1 - contact.m_pBody1->com_;

    VECTOR3 vCR0 = VECTOR3::Cross(vR0,contact.m_vNormal);
    VECTOR3 vCR1 = VECTOR3::Cross(vR1,contact.m_vNormal);

    VECTOR3 vDA0 = contact.m_pBody0->getWorldTransformedInvTensor() * vCR0;
    VECTOR3 vDA1 = contact.m_pBody1->getWorldTransformedInvTensor() * vCR1;

    contact.m_dMassNormal  = 1.0/(massinv + vCR0 * vDA0 + vCR1 * vDA1);

    contact.m_dRestitution = 0.0;

    VECTOR3 impulse0 =  contact.m_vNormal * (contact.m_dAccumulatedNormalImpulse * contact.m_pBody0->invMass_);
    VECTOR3 impulse1 = -contact.m_vNormal * (contact.m_dAccumulatedNormalImpulse * contact.m_pBody1->invMass_);

    VECTOR3 impulse  = contact.m_vNormal * contact.m_dAccumulatedNormalImpulse;

    //apply the old impulse (NOTE: not working correctly in MPI version)
#ifndef FC_MPI_SUPPORT
    contact.m_pBody0->applyImpulse(vR0, impulse,impulse0);
    contact.m_pBody1->applyImpulse(vR1,-impulse,impulse1);
#endif

    //reset the impulse
    contact.m_dAccumulatedNormalImpulse = 0.0;

    //handle the bias velocity
    //the bias velocity is proportional to the difference between the actual
    //penetration and the allowed penetration
    contact.m_dBias = m_dBiasFactor * (1.0/m_pWorld->timeControl_->GetDeltaT()) * std::min(0.0,contact.m_dPenetrationDepth+penEps);
    
    //precompute for the u-friction component
    VECTOR3 vTUR0 = VECTOR3::Cross(vR0,contact.m_vTangentU);
    VECTOR3 vTUR1 = VECTOR3::Cross(vR1,contact.m_vTangentU);    
    
    VECTOR3 vDTU0 = contact.m_pBody0->getWorldTransformedInvTensor() * vTUR0;
    VECTOR3 vDTU1 = contact.m_pBody1->getWorldTransformedInvTensor() * vTUR1;
    
    contact.m_dMassTangentU = 1.0/(massinv + vTUR0 * vDTU0 + vTUR1 * vDTU1);
    
    contact.m_dAccumulatedTangentImpulseU = 0.0;
    
    //precompute for the v-friction component
    VECTOR3 vTVR0 = VECTOR3::Cross(vR0,contact.m_vTangentV);
    VECTOR3 vTVR1 = VECTOR3::Cross(vR1,contact.m_vTangentV);    
    
    VECTOR3 vDTV0 = contact.m_pBody0->getWorldTransformedInvTensor() * vTVR0;
    VECTOR3 vDTV1 = contact.m_pBody1->getWorldTransformedInvTensor() * vTVR1;
    
    contact.m_dMassTangentV = 1.0/(massinv + vTVR0 * vDTV0 + vTVR1 * vDTV1);
    
    contact.m_dAccumulatedTangentImpulseV = 0.0;

	}

}

void CollResponseSI::ApplyImpulse(CollisionInfo &ContactInfo)
{

	double eps=0.0;

  std::vector<Contact>::iterator iter;
	for(iter=ContactInfo.m_vContacts.begin();iter!=ContactInfo.m_vContacts.end();iter++)
	{
      
        Contact &contact = *iter;

        if(contact.m_iState != CollisionInfo::TOUCHING)
          continue;

        VECTOR3 vR0 = contact.m_vPosition0 - contact.m_pBody0->com_;
        VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->com_;

        VECTOR3 relativeVelocity = (contact.m_pBody0->velocity_ + (VECTOR3::Cross(contact.m_pBody0->getAngVel(),vR0))
                                  - contact.m_pBody1->velocity_ - (VECTOR3::Cross(contact.m_pBody1->getAngVel(),vR1)));


        Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);

        //[-(1+e) * rV]/K
        Real normalImpulse    = contact.m_dMassNormal * (contact.m_dRestitution - relativeNormalVelocity);

        Real oldNormalImpulse = contact.m_dAccumulatedNormalImpulse;

        //clamp the accumulated impulse to 0
        contact.m_dAccumulatedNormalImpulse = std::max(oldNormalImpulse+normalImpulse,0.0);

        //set the impulse magnitude to the difference between
        //the accumulated impulse and the old impulse
        normalImpulse    =  contact.m_dAccumulatedNormalImpulse - oldNormalImpulse;

        VECTOR3 impulse  =  contact.m_vNormal * normalImpulse;

        VECTOR3 impulse0 =  contact.m_vNormal * (normalImpulse * contact.m_pBody0->invMass_);
        VECTOR3 impulse1 = -contact.m_vNormal * (normalImpulse * contact.m_pBody1->invMass_);

        //apply the normal impulse
        contact.m_pBody0->applyImpulse(vR0, impulse,impulse0);
        contact.m_pBody1->applyImpulse(vR1,-impulse,impulse1);

        //compute the bias impulse
        VECTOR3 relativeBias = (contact.m_pBody0->getBiasVelocity() + (VECTOR3::Cross(contact.m_pBody0->getBiasAngVel(),vR0))
                              - contact.m_pBody1->getBiasVelocity() - (VECTOR3::Cross(contact.m_pBody1->getBiasAngVel(),vR1)));

        Real relativeNormalBias = (relativeBias * contact.m_vNormal);

        Real biasImpulse = contact.m_dMassNormal * (contact.m_dBias - relativeNormalBias);

        Real oldBiasImpulse = contact.m_dBiasImpulse;

        //clamp the biasImpulse
        contact.m_dBiasImpulse = std::max(0.0,oldBiasImpulse+biasImpulse);

        biasImpulse = contact.m_dBiasImpulse - oldBiasImpulse;

        impulse = contact.m_vNormal * biasImpulse;

        impulse0 =  contact.m_vNormal * (biasImpulse * contact.m_pBody0->invMass_);
        impulse1 = -contact.m_vNormal * (biasImpulse * contact.m_pBody1->invMass_);

        //apply bias impulse
        contact.m_pBody0->applyBiasImpulse(vR0, impulse,impulse0);
        contact.m_pBody1->applyBiasImpulse(vR1,-impulse,impulse1);

        //compute the friction impulse
        //Real maxTangentImpulse = (contact.m_pBody0->friction_ * contact.m_pBody1->friction_) * contact.m_dAccumulatedNormalImpulse;
        Real maxTangentImpulse = (0.3) * contact.m_dAccumulatedNormalImpulse;

        //start with the u-tangent vector
        Real relativeTangentVelocity = relativeVelocity * contact.m_vTangentU;

        Real tangentImpulseU = contact.m_dMassTangentU * (-relativeTangentVelocity);

        //save the old accumulated impulse
        Real oldTangentImpulse = contact.m_dAccumulatedTangentImpulseU;

        //clamp the tangent impulse
        contact.m_dAccumulatedTangentImpulseU = std::max(std::min(oldTangentImpulse+tangentImpulseU,maxTangentImpulse),
                                                        -maxTangentImpulse);

        //get the delta impulse
        tangentImpulseU = contact.m_dAccumulatedTangentImpulseU - oldTangentImpulse;

        VECTOR3 tangentImpulse = contact.m_vTangentU * tangentImpulseU;

        VECTOR3 tangentImpulseU0 =  contact.m_vTangentU * (tangentImpulseU * contact.m_pBody0->invMass_);
        VECTOR3 tangentImpulseU1 = -contact.m_vTangentU * (tangentImpulseU * contact.m_pBody1->invMass_);

        //apply the tangent impulse
        contact.m_pBody0->applyImpulse(vR0, tangentImpulse,tangentImpulseU0);
        contact.m_pBody1->applyImpulse(vR1,-tangentImpulse,tangentImpulseU1);

        //same procedure for the v-tangent vector
        relativeTangentVelocity = relativeVelocity * contact.m_vTangentV;

        Real tangentImpulseV = contact.m_dMassTangentV * (-relativeTangentVelocity);

        //save the old accumulated impulse
        oldTangentImpulse = contact.m_dAccumulatedTangentImpulseV;

        //clamp the tangent impulse
        contact.m_dAccumulatedTangentImpulseV = std::max(std::min(oldTangentImpulse+tangentImpulseV,maxTangentImpulse),
                                                        -maxTangentImpulse);

        //get the delta impulse
        tangentImpulseV = contact.m_dAccumulatedTangentImpulseV - oldTangentImpulse;

        tangentImpulse = contact.m_vTangentV * tangentImpulseV;

        VECTOR3 tangentImpulseV0 =  contact.m_vTangentV * (tangentImpulseV * contact.m_pBody0->invMass_);
        VECTOR3 tangentImpulseV1 = -contact.m_vTangentV * (tangentImpulseV * contact.m_pBody1->invMass_);

        //apply the tangent impulse
        contact.m_pBody0->applyImpulse(vR0, tangentImpulse,tangentImpulseV0);
        contact.m_pBody1->applyImpulse(vR1,-tangentImpulse,tangentImpulseV1);

	}

}

void CollResponseSI::ComputeTangentSpace(const VECTOR3& normal, VECTOR3& t1, VECTOR3& t2)
{
  
  //based on the value of the z-component
  //we approximate a first tangent vector
  if(fabs(normal.z) > 0.7071067)
  {
    Real a = normal.y * normal.y + normal.z * normal.z;
    Real k = 1.0/(sqrt(a));    
    t1.x   = 0.0;
    t1.y   = -normal.z*k;
    t1.z   = normal.y *k;
    
    //compute the 2nd tangent vector by:
    //t2 = n x t1
    t2.x   = a*k;
    t2.y   = -normal.x*t1.z;
    t2.z   = normal.x *t1.y;
  }
  else
  {
    Real a = normal.x * normal.x + normal.y * normal.y;
    Real k = 1.0/(sqrt(a));    
    t1.x   = -normal.y*k;
    t1.y   = normal.x*k;
    t1.z   = 0.0;
    
    //compute the 2nd tangent vector by:
    //t2 = n x t1
    t2.x   = -normal.z*t1.y;
    t2.y   = normal.z *t1.x;
    t2.z   = a*k;
  }


}

Real CollResponseSI::ComputeDefect()
{

  CollisionHash::iterator hiter = m_pGraph->edges_->begin();
  hiter = m_pGraph->edges_->begin();
  int count = 0;
  for(;hiter!=m_pGraph->edges_->end();hiter++)
  {
    CollisionInfo &ContactInfo = *hiter;
    std::vector<Contact>::iterator iter;
	  for(iter=ContactInfo.m_vContacts.begin();iter!=ContactInfo.m_vContacts.end();iter++)
	  {
      
		  Contact &contact = *iter;

      if(contact.m_iState != CollisionInfo::TOUCHING)
        continue;

      VECTOR3 vR0 = contact.m_vPosition0 - contact.m_pBody0->com_;
      VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->com_;

      VECTOR3 relativeVelocity = (contact.m_pBody0->velocity_ + (VECTOR3::Cross(contact.m_pBody0->getAngVel(),vR0))
                                - contact.m_pBody1->velocity_ - (VECTOR3::Cross(contact.m_pBody1->getAngVel(),vR1)));

      Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);

      defect_(count++)=relativeNormalVelocity;

    }
  }
  return defect_.norm(); 
}

}
