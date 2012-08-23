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
  
CCollResponseSI::CCollResponseSI(void)
{
    m_dBiasFactor = Real(0.1);
}

CCollResponseSI::~CCollResponseSI(void)
{

}

CCollResponseSI::CCollResponseSI(std::list<CCollisionInfo> *CollInfo, CWorld *pWorld) : CCollResponse(CollInfo,pWorld)
{
    m_dBiasFactor = Real(0.1);
}

void CCollResponseSI::Solve()
{

  //return status of our solver
  int ireturnStatus;

  //number of iterations
  int iterations;

  if(this->m_pGraph->m_pEdges->IsEmpty())
    return;

  int i,j;
  Real deltaT = m_pWorld->m_pTimeControl->GetDeltaT();

  CPerfTimer timer0;
  dTimeAssembly = 0;
  dTimeSolver = 0;
  dTimeSolverPost = 0;
  dTimeAssemblyDry = 0;

  m_dBiasFactor = 0.3;

  //number of different contacts
  int nContacts=0;

  //in the SI framework we apply the external forces before
  //we call the constraint force solver
  std::vector<CRigidBody*> &vRigidBodies = m_pWorld->m_vRigidBodies;
  std::vector<CRigidBody*>::iterator rIter;

  int count = 0;
  for(rIter=vRigidBodies.begin();rIter!=vRigidBodies.end();rIter++)
  {

    CRigidBody *body = *rIter;

    if(body->m_iShape == CRigidBody::BOUNDARYBOX || !body->IsAffectedByGravity())
      continue;

    VECTOR3 &pos    = body->m_vCOM;
    VECTOR3 &vel    = body->m_vVelocity;
    //std::cout<<"body id/remoteid/velocity: "<<body->m_iID<<" "<<body->m_iRemoteID<<" "<<body->m_vVelocity;
    body->SetAngVel(VECTOR3(0,0,0));

    //velocity update
    if(body->IsAffectedByGravity())
    {
      vel += m_pWorld->GetGravityEffect(body) * m_pWorld->m_pTimeControl->GetDeltaT();
    }

    //std::cout<<"body id/remoteid/velocity: "<<body->m_iID<<" "<<body->m_iRemoteID<<" "<<body->m_vVelocity;

  }//end for

  timer0.Start();
  m_iContactPoints = 0;
  CCollisionHash::iterator hiter = m_pGraph->m_pEdges->begin();
  for(;hiter!=m_pGraph->m_pEdges->end();hiter++)
  {
    CCollisionInfo &info = *hiter;
    if(!info.m_vContacts.empty())
    {
      PreComputeConstants(info);
    }
  }

  dTimeAssemblyDry+=timer0.GetTime();

  //initialize the defect vector
  m_vDef = CVectorNr(m_iContactPoints);

  timer0.Start();
  //call the sequential impulses solver with a fixed
  //number of iterations
  for(iterations=0;iterations<100;iterations++)
  {
    //std::cout<<"Iteration: "<<iterations<<" ------------------------------------------------------"<<std::endl;
    
    for(rIter=vRigidBodies.begin();rIter!=vRigidBodies.end();rIter++)
    {

      CRigidBody *body = *rIter;

      if(!body->IsAffectedByGravity())
        continue;

      VECTOR3 &vel    = body->m_vVelocity;
      //backup the velocity
      body->m_vOldVel = body->m_vVelocity;
      //std::cout<<"body id/remoteid/velocity: "<<body->m_iID<<" "<<body->m_iRemoteID<<" "<<body->m_vVelocity;       
    }//end for
        
    hiter = m_pGraph->m_pEdges->begin();
    for(;hiter!=m_pGraph->m_pEdges->end();hiter++)
    {
      CCollisionInfo &info = *hiter;
      if(!info.m_vContacts.empty())
      {
        ApplyImpulse(info);
      }
    }
          
    //we now have to synchronize the remote bodies
    for(int j=0;j<1;j++)//myWorld.m_myParInfo.m_Groups.size();j++)
    {
      if(m_pWorld->m_myParInfo.GetID() == m_pWorld->m_myParInfo.m_Groups[j].m_iRoot)
      {
        //the root of the group computes the maxium of remote bodies within the group
        int nBodies = m_pWorld->m_pSubBoundary->GetMaxRemotes();

        //1loop over all elements of the group

        //2map group element to subdomain neighbor

        //3get the number of remotes for the subdomain neighbor

        //this array is put into the sendbuffer parameter of the MPI_Gather function
        int *sendbuffer_map = new int[nBodies];

        //this array is the receivebuffer parameter of the MPI_Gather function and
        //contains the mapping from the communicated array into the root's m_vRigidBodies vector
        int *rec_map=new int[m_pWorld->m_myParInfo.m_Groups[j].m_iSize*nBodies];

        //this array is put into the sendbuffer parameter of the MPI_Gather function
        //for the root of the group
        Real *sendbuffer_updates = new Real[3*nBodies];

        //we compute the size of the array that receives the velocity updates: 3*group_size*max_bodies_in_group
        int  rec_size = 3*m_pWorld->m_myParInfo.m_Groups[j].m_iSize*nBodies;

        //allocate memory for the velocity updates
        Real *rec_updates=new Real[rec_size];

        //memory for communication the updated velocity back to the remote domain
        Real *send_sync = new Real[3*nBodies*m_pWorld->m_myParInfo.m_Groups[j].m_iSize];
        Real *rec_sync  = new Real[3*nBodies];

        //In the very first communication step the root of the group sends desired length of
        //the communication arrays to the other members of the group
        MPI_Bcast(&nBodies, 1, MPI_INT,
                  m_pWorld->m_myParInfo.m_Groups[j].m_iRoot,
                  m_pWorld->m_myParInfo.m_AllComm[m_pWorld->m_myParInfo.m_Groups[j].m_iRoot]);

        //In the first MPI_Gather we gather all velocity update at the root of the group
        MPI_Gather(sendbuffer_updates,
                   3*nBodies,
                   MPI_DOUBLE,
                   rec_updates,
                   3*nBodies,
                   MPI_DOUBLE,
                   m_pWorld->m_myParInfo.m_Groups[j].m_iRoot,
                   m_pWorld->m_myParInfo.m_AllComm[m_pWorld->m_myParInfo.m_Groups[j].m_iRoot]);

        //In the second MPI_Gather we gather the mapping from the rec_updates to the root's m_vRigidBodies vector
        MPI_Gather(sendbuffer_map,
                   nBodies,
                   MPI_INT,
                   rec_map,
                   nBodies,
                   MPI_INT,
                   m_pWorld->m_myParInfo.m_Groups[j].m_iRoot,
                   m_pWorld->m_myParInfo.m_AllComm[m_pWorld->m_myParInfo.m_Groups[j].m_iRoot]);

        //loop over all members of the group
        for(int i=1;i<m_pWorld->m_myParInfo.m_Groups[j].m_iSize;i++)
        {
          //and apply all velocity updates
          for(int k=0;k<nBodies;k++)
          {
            CRigidBody *body     = m_pWorld->m_vRigidBodies[rec_map[i*nBodies+k]];
            body->m_vVelocity.x += rec_updates[i*3*nBodies+3*k];
            body->m_vVelocity.y += rec_updates[i*3*nBodies+3*k+1];
            body->m_vVelocity.z += rec_updates[i*3*nBodies+3*k+2];
          }
        }

        // loop over all members of the group
        for(int i=1;i<m_pWorld->m_myParInfo.m_Groups[j].m_iSize;i++)
        {
          // write back velocity
          for(int k=0;k<nBodies;k++)
          {
            //we need to map the
            CRigidBody *body = m_pWorld->m_vRigidBodies[rec_map[i*nBodies+k]];
            send_sync[i*3*nBodies+3*k]   = body->m_vVelocity.x;
            send_sync[i*3*nBodies+3*k+1] = body->m_vVelocity.y;
            send_sync[i*3*nBodies+3*k+2] = body->m_vVelocity.z;
          }
        }

        // in the last communication step we scatter the updated velocity to the other
        // members of the group
        MPI_Scatter(send_sync,
                    3*nBodies,
                    MPI_DOUBLE,
                    rec_sync,
                    3*nBodies,
                    MPI_DOUBLE,
                    m_pWorld->m_myParInfo.m_Groups[j].m_iRoot,
                    m_pWorld->m_myParInfo.m_AllComm[m_pWorld->m_myParInfo.m_Groups[j].m_iRoot]);

        //free the memory used for communication
        delete[] sendbuffer_map;
        delete[] sendbuffer_updates;
        delete[] rec_updates;
        delete[] rec_map;
        delete[] send_sync;
        delete[] rec_sync;
      }
      else
      {
        //we enter the else branch in case that the process is not the
        //root of the current group
        int nBodies = 0;
        //std::cout<<"Number of remotes in 1 "<<nBodies<<std::endl;

        //In the very first communication step the root of the group sends desired length of
        //the communication arrays to the other members of the group
        MPI_Bcast(&nBodies, 1, MPI_INT,
                  m_pWorld->m_myParInfo.m_Groups[j].m_iRoot,
                  m_pWorld->m_myParInfo.m_AllComm[m_pWorld->m_myParInfo.m_Groups[j].m_iRoot]);

        Real *buffer_updates = new Real[3*nBodies];

        int *sendbuffer_map  = new int[nBodies];

        Real *send_sync = new Real[3*nBodies*m_pWorld->m_myParInfo.m_Groups[j].m_iSize];

        //at first we calculate the velocity updates and save them in our sendbuffer
        for(int k=0;k<nBodies;k++)
        {
          sendbuffer_map[k]=m_pWorld->m_pSubBoundary->m_iRemoteIDs[j][k];
          CRigidBody *body = m_pWorld->m_vRigidBodies[m_pWorld->m_pSubBoundary->m_iRemoteBodies[j][k]];
          buffer_updates[3*k]   = body->m_vVelocity.x - body->m_vOldVel.x;
          buffer_updates[3*k+1] = body->m_vVelocity.y - body->m_vOldVel.y;
          buffer_updates[3*k+2] = body->m_vVelocity.z - body->m_vOldVel.z;
          //std::cout<<"velocity difference: "<<body->m_vVelocity<<body->m_vOldVel;
          //std::cout<<"1:"<<"body id/remoteid/velocity: "<<body->m_iID<<" "<<body->m_iRemoteID<<" "<<body->m_vVelocity;
        }

        Real *rec_updates = NULL;

        int    *rec_map   = NULL;

        // In the first MPI_Gather we gather all velocity update at the root of the group
        MPI_Gather(buffer_updates,
                   3*nBodies,
                   MPI_DOUBLE,
                   rec_updates,
                   3*nBodies,
                   MPI_DOUBLE,
                   m_pWorld->m_myParInfo.m_Groups[j].m_iRoot,
                   m_pWorld->m_myParInfo.m_AllComm[m_pWorld->m_myParInfo.m_Groups[j].m_iRoot]);

        // In the second MPI_Gather we send the mapping from the diffs array to the root's m_vRigidBodies vector
        MPI_Gather(sendbuffer_map,
                   nBodies,
                   MPI_INT,
                   rec_map,
                   nBodies,
                   MPI_INT,
                   m_pWorld->m_myParInfo.m_Groups[j].m_iRoot,
                   m_pWorld->m_myParInfo.m_AllComm[m_pWorld->m_myParInfo.m_Groups[j].m_iRoot]);

        // in the last communication step we get the corrected velocity from the root
        MPI_Scatter(send_sync,
                    3*nBodies,
                    MPI_DOUBLE,
                    buffer_updates,
                    3*nBodies,
                    MPI_DOUBLE,
                    m_pWorld->m_myParInfo.m_Groups[j].m_iRoot,
                    m_pWorld->m_myParInfo.m_AllComm[m_pWorld->m_myParInfo.m_Groups[j].m_iRoot]);

        // set the corrected velocity from the root
        for(int k=0;k<nBodies;k++)
        {
          CRigidBody *body = m_pWorld->m_vRigidBodies[m_pWorld->m_pSubBoundary->m_iRemoteBodies[j][k]];
          body->m_vVelocity.x = buffer_updates[3*k];
          body->m_vVelocity.y = buffer_updates[3*k+1];
          body->m_vVelocity.z = buffer_updates[3*k+2];
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

          //backup the velocity
          body->m_vOldVel = body->m_vVelocity;

          //std::cout<<"body id/remoteid/velocity: "<<body->m_iID<<" "<<body->m_iRemoteID<<" "<<body->m_vVelocity;
        }//end for

        //free the buffer arrays
        delete[] buffer_updates;
        delete[] sendbuffer_map;
        delete[] send_sync;

      }

    }
    //std::cout<<"Iteration: "<<iterations <<" "<<ComputeDefect()<<std::endl;

  }//end for all groups

  dTimeSolver+=timer0.GetTime();

}//end Solve

void CCollResponseSI::PreComputeConstants(CCollisionInfo &ContactInfo)
{
  Real penEps = 0.0006;
  std::vector<CContact>::iterator iter;
	for(iter=ContactInfo.m_vContacts.begin();iter!=ContactInfo.m_vContacts.end();iter++)
	{
      
    CContact &contact = *iter;

    if(contact.m_iState != CCollisionInfo::TOUCHING)
      continue;

    m_iContactPoints++;

    ComputeTangentSpace(contact.m_vNormal,contact.m_vTangentU,contact.m_vTangentV);
    
    //the massinverse is needed
    Real massinv = contact.m_pBody0->m_dInvMass + contact.m_pBody1->m_dInvMass;

    VECTOR3 vR0  = contact.m_vPosition0 - contact.m_pBody0->m_vCOM;
    VECTOR3 vR1  = contact.m_vPosition1 - contact.m_pBody1->m_vCOM;

    VECTOR3 vCR0 = VECTOR3::Cross(vR0,contact.m_vNormal);
    VECTOR3 vCR1 = VECTOR3::Cross(vR1,contact.m_vNormal);

    VECTOR3 vDA0 = contact.m_pBody0->GetWorldTransformedInvTensor() * vCR0;
    VECTOR3 vDA1 = contact.m_pBody1->GetWorldTransformedInvTensor() * vCR1;

    contact.m_dMassNormal  = 1.0/(massinv + vCR0 * vDA0 + vCR1 * vDA1);

    contact.m_dRestitution = 0.0;

    VECTOR3 impulse0 =  contact.m_vNormal * (contact.m_dAccumulatedNormalImpulse * contact.m_pBody0->m_dInvMass);
    VECTOR3 impulse1 = -contact.m_vNormal * (contact.m_dAccumulatedNormalImpulse * contact.m_pBody1->m_dInvMass);

    VECTOR3 impulse  = contact.m_vNormal * contact.m_dAccumulatedNormalImpulse;

    //apply the old impulse (NOTE: not working correctly in MPI version)
#ifndef FC_MPI_SUPPORT
    contact.m_pBody0->ApplyImpulse(vR0, impulse,impulse0);
    contact.m_pBody1->ApplyImpulse(vR1,-impulse,impulse1);
#endif

    //reset the impulse
    contact.m_dAccumulatedNormalImpulse = 0.0;

    //handle the bias velocity
    //the bias velocity is proportional to the difference between the actual
    //penetration and the allowed penetration
    contact.m_dBias = m_dBiasFactor * (1.0/m_pWorld->m_pTimeControl->GetDeltaT()) * std::min(0.0,contact.m_dPenetrationDepth+penEps);
    
    //precompute for the u-friction component
    VECTOR3 vTUR0 = VECTOR3::Cross(vR0,contact.m_vTangentU);
    VECTOR3 vTUR1 = VECTOR3::Cross(vR1,contact.m_vTangentU);    
    
    VECTOR3 vDTU0 = contact.m_pBody0->GetWorldTransformedInvTensor() * vTUR0;
    VECTOR3 vDTU1 = contact.m_pBody1->GetWorldTransformedInvTensor() * vTUR1;
    
    contact.m_dMassTangentU = 1.0/(massinv + vTUR0 * vDTU0 + vTUR1 * vDTU1);
    
    contact.m_dAccumulatedTangentImpulseU = 0.0;
    
    //precompute for the v-friction component
    VECTOR3 vTVR0 = VECTOR3::Cross(vR0,contact.m_vTangentV);
    VECTOR3 vTVR1 = VECTOR3::Cross(vR1,contact.m_vTangentV);    
    
    VECTOR3 vDTV0 = contact.m_pBody0->GetWorldTransformedInvTensor() * vTVR0;
    VECTOR3 vDTV1 = contact.m_pBody1->GetWorldTransformedInvTensor() * vTVR1;
    
    contact.m_dMassTangentV = 1.0/(massinv + vTVR0 * vDTV0 + vTVR1 * vDTV1);
    
    contact.m_dAccumulatedTangentImpulseV = 0.0;

	}

}

void CCollResponseSI::ApplyImpulse(CCollisionInfo &ContactInfo)
{

	double eps=0.0;

    std::vector<CContact>::iterator iter;
	for(iter=ContactInfo.m_vContacts.begin();iter!=ContactInfo.m_vContacts.end();iter++)
	{
      
        CContact &contact = *iter;

        if(contact.m_iState != CCollisionInfo::TOUCHING)
          continue;

        VECTOR3 vR0 = contact.m_vPosition0 - contact.m_pBody0->m_vCOM;
        VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->m_vCOM;

        VECTOR3 relativeVelocity = (contact.m_pBody0->m_vVelocity + (VECTOR3::Cross(contact.m_pBody0->GetAngVel(),vR0))
                                  - contact.m_pBody1->m_vVelocity - (VECTOR3::Cross(contact.m_pBody1->GetAngVel(),vR1)));


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

        VECTOR3 impulse0 =  contact.m_vNormal * (normalImpulse * contact.m_pBody0->m_dInvMass);
        VECTOR3 impulse1 = -contact.m_vNormal * (normalImpulse * contact.m_pBody1->m_dInvMass);

        //apply the normal impulse
        contact.m_pBody0->ApplyImpulse(vR0, impulse,impulse0);
        contact.m_pBody1->ApplyImpulse(vR1,-impulse,impulse1);

        //compute the bias impulse
        VECTOR3 relativeBias = (contact.m_pBody0->GetBiasVelocity() + (VECTOR3::Cross(contact.m_pBody0->GetBiasAngVel(),vR0))
                              - contact.m_pBody1->GetBiasVelocity() - (VECTOR3::Cross(contact.m_pBody1->GetBiasAngVel(),vR1)));

        Real relativeNormalBias = (relativeBias * contact.m_vNormal);

        Real biasImpulse = contact.m_dMassNormal * (contact.m_dBias - relativeNormalBias);

        Real oldBiasImpulse = contact.m_dBiasImpulse;

        //clamp the biasImpulse
        contact.m_dBiasImpulse = std::max(0.0,oldBiasImpulse+biasImpulse);

        biasImpulse = contact.m_dBiasImpulse - oldBiasImpulse;

        impulse = contact.m_vNormal * biasImpulse;

        impulse0 =  contact.m_vNormal * (biasImpulse * contact.m_pBody0->m_dInvMass);
        impulse1 = -contact.m_vNormal * (biasImpulse * contact.m_pBody1->m_dInvMass);

        //apply bias impulse
        contact.m_pBody0->ApplyBiasImpulse(vR0, impulse,impulse0);
        contact.m_pBody1->ApplyBiasImpulse(vR1,-impulse,impulse1);

        //compute the friction impulse
        Real maxTangentImpulse = (contact.m_pBody0->m_dFriction * contact.m_pBody1->m_dFriction) * contact.m_dAccumulatedNormalImpulse;

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

        VECTOR3 tangentImpulseU0 =  contact.m_vTangentU * (tangentImpulseU * contact.m_pBody0->m_dInvMass);
        VECTOR3 tangentImpulseU1 = -contact.m_vTangentU * (tangentImpulseU * contact.m_pBody1->m_dInvMass);

        //apply the tangent impulse
        contact.m_pBody0->ApplyImpulse(vR0, tangentImpulse,tangentImpulseU0);
        contact.m_pBody1->ApplyImpulse(vR1,-tangentImpulse,tangentImpulseU1);

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

        VECTOR3 tangentImpulseV0 =  contact.m_vTangentV * (tangentImpulseV * contact.m_pBody0->m_dInvMass);
        VECTOR3 tangentImpulseV1 = -contact.m_vTangentV * (tangentImpulseV * contact.m_pBody1->m_dInvMass);

        //apply the tangent impulse
        contact.m_pBody0->ApplyImpulse(vR0, tangentImpulse,tangentImpulseV0);
        contact.m_pBody1->ApplyImpulse(vR1,-tangentImpulse,tangentImpulseV1);

	}

}

void CCollResponseSI::ComputeTangentSpace(const VECTOR3& normal, VECTOR3& t1, VECTOR3& t2)
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

Real CCollResponseSI::ComputeDefect()
{

  CCollisionHash::iterator hiter = m_pGraph->m_pEdges->begin();
  hiter = m_pGraph->m_pEdges->begin();
  int count = 0;
  for(;hiter!=m_pGraph->m_pEdges->end();hiter++)
  {
    CCollisionInfo &ContactInfo = *hiter;
    std::vector<CContact>::iterator iter;
	  for(iter=ContactInfo.m_vContacts.begin();iter!=ContactInfo.m_vContacts.end();iter++)
	  {
      
		  CContact &contact = *iter;

      if(contact.m_iState != CCollisionInfo::TOUCHING)
        continue;

      VECTOR3 vR0 = contact.m_vPosition0 - contact.m_pBody0->m_vCOM;
      VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->m_vCOM;

      VECTOR3 relativeVelocity = (contact.m_pBody0->m_vVelocity + (VECTOR3::Cross(contact.m_pBody0->GetAngVel(),vR0))
                                - contact.m_pBody1->m_vVelocity - (VECTOR3::Cross(contact.m_pBody1->GetAngVel(),vR1)));

      Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);

      m_vDef(count++)=relativeNormalVelocity;
    }
  }
  return m_vDef.norm(); 
}

}
