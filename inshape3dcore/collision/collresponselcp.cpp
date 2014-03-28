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

#include "collresponselcp.h"
#include <3dmodel.h>
#include <3dmesh.h>
#include <set>
#include <iostream>
#include <linearsolverlu.h>
#include <linearsolvergauss.h>
#include <lemkesolver.h>
#include <map>
#include <rigidbody.h>
#include <world.h>
#include <pathsolver.h>
#include <globalheader.h>
#include <rigidbodymotion.h>
#include <lcpsolvergaussseidel.h>
#include <perftimer.h>
#include <lcpsolverjacobi.h>
#ifdef FC_OPENMP
#include <omp.h>
#endif

namespace i3d {

CollResponseLcp::CollResponseLcp(void)
{
  m_pSolver = NULL;
}

CollResponseLcp::~CollResponseLcp(void)
{
  delete m_pSolver;
}

CollResponseLcp::CollResponseLcp(std::list<CollisionInfo> *CollInfo, World *pWorld) : CollResponse(CollInfo,pWorld)
{

}

void CollResponseLcp::InitSolverPGS(int maxIterations, Real omega)
{
 m_pSolver = new CLcpSolverGaussSeidel<Real>(maxIterations,omega); 
}

void CollResponseLcp::InitSolverPJA(int maxIterations, Real omega)
{
 m_pSolver = new CLcpSolverJacobi<Real>(maxIterations,1.0); 
}

void CollResponseLcp::Solve()
{
  //return status of our solver
  int ireturnStatus;

  CPerfTimer timer0;
  dTimeAssembly = 0;
  dTimeSolver = 0;
  dTimeSolverPost = 0;
  dTimeAssemblyDry = 0;
  std::list<CollisionInfo>::iterator Iter;
  std::vector<Contact>::iterator cIter;
  
  if(this->m_pGraph->edges_->IsEmpty())
    return;

  int i,j;
  Real deltaT = this->m_pWorld->timeControl_->GetDeltaT();
  //number of different contacts
  int nContacts=0;
  std::vector<Contact*> vContacts;

  CollisionHash::iterator hiter = m_pGraph->edges_->begin();
  for(;hiter!=m_pGraph->edges_->end();hiter++)
  {
    CollisionInfo &info = *hiter;
    for(cIter=info.m_vContacts.begin();cIter!=info.m_vContacts.end();cIter++)
    {
      Contact &contact = *cIter;
      if(contact.m_iState == CollisionInfo::TOUCHING)
        vContacts.push_back(&contact);
    }
  }
  
  nContacts = vContacts.size();

  if(nContacts == 0)
    return;

  //Initialize matrix and vectors
  //CMatrixNxN<double> M(nContacts,nContacts);
  CVectorN<double> W(nContacts);
  CVectorN<double> Q(nContacts);
  CVectorN<double> Z(nContacts);

  std::vector<Contact*>::iterator pIter;
  //get the forces from the contact cache
  for(pIter=vContacts.begin(),i=0;pIter!=vContacts.end();pIter++,i++)
  {
    Contact *contact = *pIter;
    Z(i)=contact->m_dAccumulatedNormalImpulse;
  }

  timer0.Start();
  //assemble the matrix
  //AssembleVelocityBased(M,Q,vContacts);
  int *rowPointer = new int[nContacts+1];
  int entries = ComputeMatrixStructure(vContacts,rowPointer);
  dTimeAssemblyDry+=timer0.GetTime();

  CMatrixCSR<Real> matrix(vContacts.size(),entries,rowPointer);

  timer0.Start();
  AssembleVelocityBasedCSR(matrix,Q,vContacts);
  dTimeAssembly+=timer0.GetTime();

  Q.invert();

  //solve the lcp
  m_pSolver->SetMatrix(matrix);
  m_pSolver->SetQWZ(Q,W,Z);


  timer0.Start();
  m_pSolver->Solve();
  dTimeSolver+=timer0.GetTime();

  //matrix.OutputMatrix();
  // Q.OutputVector();
  // Z.OutputVector();
  m_iContactPoints = nContacts;

  timer0.Start();
  //apply the impluses calculated by
  //the lcp solver, so after application
  //of the impulses all vn >= 0
  ApplyImpulse(nContacts,Z,vContacts);
  //std::cout<<"Residual: "<<m_pSolver->m_dResidual<<" Number of iterations used: "<<m_pSolver->m_iIterationsUsed<<"\n";
  //std::cout<<"Number of zero entries: "<<M.NumZeros()<<std::endl;
  m_pSolver->CleanUp();
  dTimeSolverPost+=timer0.GetTime();

}//end function

void CollResponseLcp::AssembleVelocityBased(CMatrixNxN<double> &M, CVectorN<double> &Q, std::vector<Contact*> &vContacts)
{

  std::vector<Contact*>::iterator cIter;
  int nContacts = vContacts.size();
  int i,j;
  Real dSign0,dSign1;
  //loop over all contacts
  //every contact will produce a row in the matrix M
  //for(cIter=vContacts.begin(),i=0;cIter!=vContacts.end();cIter++,i++)
  double wall_timer;
  i=0;
#ifdef FC_OPENMP
#pragma omp parallel default(shared) private(i,j,wall_timer,dSign0,dSign1) num_threads(2)
{
  wall_timer = omp_get_wtime();
  #pragma omp for
#endif
  for(i=0;i<nContacts;i++)
  {
    Contact &contact = *(vContacts[i]);
    //average the restitution
    Real restitution = (contact.m_pBody0->restitution_ * contact.m_pBody1->restitution_);
    VECTOR3 angVel0 = contact.m_pBody0->getAngVel();
    VECTOR3 angVel1 = contact.m_pBody1->getAngVel();

    //get the world-transformed inertia tensor
    MATRIX3X3 mInvInertiaTensor0 = contact.m_pBody0->getWorldTransformedInvTensor();
    MATRIX3X3 mInvInertiaTensor1 = contact.m_pBody1->getWorldTransformedInvTensor();
    VECTOR3 vR0 = contact.m_vPosition0-contact.m_pBody0->com_;
    VECTOR3 vR1 = contact.m_vPosition1-contact.m_pBody1->com_;
    VECTOR3 vAcc0(0,0,0);
    VECTOR3 vAcc1(0,0,0);

    VECTOR3 relativeVelocity = 
      (contact.m_pBody0->velocity_ + (VECTOR3::Cross(angVel0,vR0))
      - contact.m_pBody1->velocity_ - (VECTOR3::Cross(angVel1,vR1)));

    Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);


    if(contact.m_pBody0->getShape() == RigidBody::BOUNDARYBOX || !contact.m_pBody0->isAffectedByGravity())
      vAcc0=VECTOR3(0,0,0);
    else
    {
      //gravity + other external acceleration
      vAcc0  = m_pWorld->getGravityEffect(contact.m_pBody0);
      vAcc0 += contact.m_pBody0->force_ * contact.m_pBody0->invMass_;
      vAcc0 += VECTOR3::Cross(contact.m_pBody0->getWorldTransformedInvTensor() * contact.m_pBody0->torque_,vR0);
    }

    if(contact.m_pBody1->getShape() == RigidBody::BOUNDARYBOX || !contact.m_pBody1->isAffectedByGravity())
      vAcc1=VECTOR3(0,0,0);
    else
    {
      //gravity + other external acceleration      
      vAcc1  = m_pWorld->getGravityEffect(contact.m_pBody1);
      vAcc1 += contact.m_pBody1->force_ * contact.m_pBody1->invMass_;
      vAcc1 += VECTOR3::Cross(contact.m_pBody1->getWorldTransformedInvTensor() * contact.m_pBody1->torque_,vR1);
    }

    Q(i)  = (1+restitution) * relativeNormalVelocity + contact.m_vNormal * m_pWorld->timeControl_->GetDeltaT() * (vAcc0 - vAcc1);   
    
    //assemble the diagonal element
    //the diagonal element of a contact has always two parts,
    //one for the first body and one for the second
    //only the point of application on the body
    //and the direction of the contact normal differ
    Real term0 = contact.m_pBody0->invMass_;
    Real angularTerm0 = contact.m_vNormal * ((VECTOR3::Cross(mInvInertiaTensor0 * VECTOR3::Cross(vR0,contact.m_vNormal),vR0)));

    Real term1 = contact.m_pBody1->invMass_;
    Real angularTerm1 = contact.m_vNormal * ((VECTOR3::Cross(mInvInertiaTensor1 * VECTOR3::Cross(vR1,contact.m_vNormal),vR1)));

    //on the diagonal we add the terms
    //that means the diagonal element is N_i * [(m_a^-1*N_i + N_i * (J_a^-1*(r_ia x N_i)) x r_ia) + (m_b^-1*N_i + N_i * (J_b^-1*(r_ib x N_i)) x r_ib)]
    M(i,i)       =  term0 + angularTerm0 + term1 + angularTerm1;

    //assemble the remaining elements in the row
    //may have one part, two parts or it can be just 0
    for(j=i+1;j<nContacts;j++)
    {
      //initialize the entry with zero
      //the entry is non-zero only in case the
      //jth-contact includes the body0 or body1 of the
      //ith-contact
      M(i,j)=0.0;
      VECTOR3 vTerm0=VECTOR3(0,0,0);
      VECTOR3 vAngularTerm0=VECTOR3(0,0,0);
      VECTOR3 vTerm1=VECTOR3(0,0,0);
      VECTOR3 vAngularTerm1=VECTOR3(0,0,0);

      //assemble off-diagonal
      //check if body 0 is in the j-th contact
      if((dSign0=vContacts[j]->GetSign(contact.m_pBody0)) != 0.0)
      {
        VECTOR3 vRj = (dSign0 > Real(0.0)) ? vContacts[j]->m_vPosition0-contact.m_pBody0->com_ : vContacts[j]->m_vPosition1-contact.m_pBody0->com_;
        //VECTOR3 vRj = (dSign0 > Real(0.0)) ? vContacts[j].m_vPosition0 : vContacts[j].m_vPosition1;
        vTerm0 = contact.m_pBody0->invMass_ * vContacts[j]->m_vNormal;
        vAngularTerm0 =(VECTOR3::Cross(mInvInertiaTensor0 * VECTOR3::Cross(vRj,vContacts[j]->m_vNormal),vR0));
      }

      //check if body 1 is in the j-th contact
      if((dSign1=vContacts[j]->GetSign(contact.m_pBody1)) != 0.0)
      {
        VECTOR3 vRj = (dSign1 > Real(0.0)) ? vContacts[j]->m_vPosition0-contact.m_pBody1->com_ : vContacts[j]->m_vPosition1-contact.m_pBody1->com_;
        //VECTOR3 vRj = (dSign1 > Real(0.0)) ? vContacts[j].m_vPosition0 : vContacts[j].m_vPosition1;
        vTerm1 = ((contact.m_pBody1->invMass_ * vContacts[j]->m_vNormal));
        vAngularTerm1 = (VECTOR3::Cross(mInvInertiaTensor1 * VECTOR3::Cross(vRj,vContacts[j]->m_vNormal),vR1));
      }

      M(i,j) = contact.m_vNormal * (dSign0 * (vTerm0 + vAngularTerm0) - dSign1 * (vTerm1 + vAngularTerm1));
      //M(i,j) = contact.m_vNormal * (dSign0 * (vTerm0) - dSign1 * (vTerm1));

    }//end for j

  //printf("Thread: %i, row %i column: %i \n",omp_get_thread_num(),i,j);
  }//end for i
#ifdef FC_OPENMP
  printf("time on wall %i = %.3f\n",omp_get_thread_num(),omp_get_wtime() - wall_timer);
}
#endif

  //the matrix is symmetric so we now copy the
  //upper right part of the matrix to
  //the lower left
  for(i=0;i<nContacts;i++)
  {
    for(j=i-1;j>=0;j--)
    {
      M(i,j)=M(j,i);
    }//end for j
  }

}

void CollResponseLcp::AssembleVelocityBasedCSR(CMatrixCSR<double> &M, CVectorN<double> &Q, std::vector<Contact*> &vContacts)
{
    std::vector<Contact*>::iterator cIter;
    int nContacts = vContacts.size();
    int i,j,index;
    Real dSign0,dSign1;
    //loop over all contacts
    //every contact will produce a row in the matrix M
    //for(cIter=vContacts.begin(),i=0;cIter!=vContacts.end();cIter++,i++)
    double wall_timer;
    i=0;
#ifdef FC_OPENMP
#pragma omp parallel default(shared) private(i,j,index,wall_timer,dSign0,dSign1) num_threads(4)
{
    wall_timer = omp_get_wtime();
    #pragma omp for
#endif
    for(i=0;i<nContacts;i++)
    {
      Contact &contact = *(vContacts[i]);

      index=M.m_iRowPtr[i];
      //printf("Thread: %i, row %i index: %i \n",omp_get_thread_num(),i,index);

      //average the restitution
      Real restitution = (contact.m_pBody0->restitution_ * contact.m_pBody1->restitution_);
      VECTOR3 angVel0 = contact.m_pBody0->getAngVel();
      VECTOR3 angVel1 = contact.m_pBody1->getAngVel();

      //get the world-transformed inertia tensor
      MATRIX3X3 mInvInertiaTensor0 = contact.m_pBody0->getWorldTransformedInvTensor();
      MATRIX3X3 mInvInertiaTensor1 = contact.m_pBody1->getWorldTransformedInvTensor();
      VECTOR3 vR0 = contact.m_vPosition0-contact.m_pBody0->com_;
      VECTOR3 vR1 = contact.m_vPosition1-contact.m_pBody1->com_;
      VECTOR3 vAcc0(0,0,0);
      VECTOR3 vAcc1(0,0,0);

      VECTOR3 relativeVelocity =
        (contact.m_pBody0->velocity_ + (VECTOR3::Cross(angVel0,vR0))
        - contact.m_pBody1->velocity_ - (VECTOR3::Cross(angVel1,vR1)));

      Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);

      //loop over the row
      for(j=0;j<i;j++)
      {
        //initialize the entry with zero
        //the entry is non-zero only in case the
        //jth-contact includes the body0 or body1 of the
        //ith-contact
        bool found = false;
        VECTOR3 vTerm0=VECTOR3(0,0,0);
        VECTOR3 vAngularTerm0=VECTOR3(0,0,0);
        VECTOR3 vTerm1=VECTOR3(0,0,0);
        VECTOR3 vAngularTerm1=VECTOR3(0,0,0);

        //assemble off-diagonal
        //check if body 0 is in the j-th contact
        if((dSign0=vContacts[j]->GetSign(contact.m_pBody0)) != 0.0)
        {
          //a non-zero entry will only be created if the corresponding body
          //is affected by gravity(the inverse mass is non-zero)
          if(contact.m_pBody0->isAffectedByGravity())
          {
            VECTOR3 vRj = (dSign0 > Real(0.0)) ? vContacts[j]->m_vPosition0-contact.m_pBody0->com_ : vContacts[j]->m_vPosition1-contact.m_pBody0->com_;
            //VECTOR3 vRj = (dSign0 > Real(0.0)) ? vContacts[j].m_vPosition0 : vContacts[j].m_vPosition1;
            vTerm0 = contact.m_pBody0->invMass_ * vContacts[j]->m_vNormal;
            vAngularTerm0 =(VECTOR3::Cross(mInvInertiaTensor0 * VECTOR3::Cross(vRj,vContacts[j]->m_vNormal),vR0));
            found=true;
          }
        }
        //check if body 1 is in the j-th contact
        if((dSign1=vContacts[j]->GetSign(contact.m_pBody1)) != 0.0)
        {
          //a non-zero entry will only be created if the corresponding body
          //is affected by gravity(the inverse mass is non-zero)
          if(contact.m_pBody1->isAffectedByGravity())
          {
            found=true;
           VECTOR3 vRj = (dSign1 > Real(0.0)) ? vContacts[j]->m_vPosition0-contact.m_pBody1->com_ : vContacts[j]->m_vPosition1-contact.m_pBody1->com_;
           //VECTOR3 vRj = (dSign1 > Real(0.0)) ? vContacts[j].m_vPosition0 : vContacts[j].m_vPosition1;
           vTerm1 = ((contact.m_pBody1->invMass_ * vContacts[j]->m_vNormal));
           vAngularTerm1 = (VECTOR3::Cross(mInvInertiaTensor1 * VECTOR3::Cross(vRj,vContacts[j]->m_vNormal),vR1));
          }

        }

        if(found)
        {
          Real val = contact.m_vNormal * (dSign0 * (vTerm0 + vAngularTerm0) - dSign1 * (vTerm1 + vAngularTerm1));
            M.m_dValues[index] = val;
            M.m_iColInd[index] = j;
            index++;
        }
      }//end for j

      if(!contact.m_pBody0->isAffectedByGravity())
        vAcc0=VECTOR3(0,0,0);
      else
      {
        //gravity + other external acceleration
        vAcc0  = m_pWorld->getGravityEffect(contact.m_pBody0);
        vAcc0 += contact.m_pBody0->force_ * contact.m_pBody0->invMass_;
        vAcc0 += VECTOR3::Cross(contact.m_pBody0->getWorldTransformedInvTensor() * contact.m_pBody0->torque_,vR0);
      }

      if(!contact.m_pBody1->isAffectedByGravity())
        vAcc1=VECTOR3(0,0,0);
      else
      {
        //gravity + other external acceleration
        vAcc1  = m_pWorld->getGravityEffect(contact.m_pBody1);
        vAcc1 += contact.m_pBody1->force_ * contact.m_pBody1->invMass_;
        vAcc1 += VECTOR3::Cross(contact.m_pBody1->getWorldTransformedInvTensor() * contact.m_pBody1->torque_,vR1);
      }

      Q(i)  = (1+restitution) * relativeNormalVelocity + contact.m_vNormal * m_pWorld->timeControl_->GetDeltaT() * (vAcc0 - vAcc1);

      //assemble the diagonal element
      //the diagonal element of a contact has always two parts,
      //one for the first body and one for the second
      //only the point of application on the body
      //and the direction of the contact normal differ
      Real term0 = contact.m_pBody0->invMass_;
      Real angularTerm0 = contact.m_vNormal * ((VECTOR3::Cross(mInvInertiaTensor0 * VECTOR3::Cross(vR0,contact.m_vNormal),vR0)));

      Real term1 = contact.m_pBody1->invMass_;
      Real angularTerm1 = contact.m_vNormal * ((VECTOR3::Cross(mInvInertiaTensor1 * VECTOR3::Cross(vR1,contact.m_vNormal),vR1)));

      //on the diagonal we add the terms
      //that means the diagonal element is N_i * [(m_a^-1*N_i + N_i * (J_a^-1*(r_ia x N_i)) x r_ia) + (m_b^-1*N_i + N_i * (J_b^-1*(r_ib x N_i)) x r_ib)]
      M.m_dValues[index] = term0 + angularTerm0 + term1 + angularTerm1;
      M.m_iColInd[index] = i;

      //increase the index into the entries array
      index++;

      //assemble the remaining elements in the row
      //may have one part, two parts or it can be just 0
      for(j=i+1;j<nContacts;j++)
      {
        //initialize the entry with zero
        //the entry is non-zero only in case the
        //jth-contact includes the body0 or body1 of the
        //ith-contact
        bool found = false;
        VECTOR3 vTerm0=VECTOR3(0,0,0);
        VECTOR3 vAngularTerm0=VECTOR3(0,0,0);
        VECTOR3 vTerm1=VECTOR3(0,0,0);
        VECTOR3 vAngularTerm1=VECTOR3(0,0,0);

        //assemble off-diagonal
        //check if body 0 is in the j-th contact
        if((dSign0=vContacts[j]->GetSign(contact.m_pBody0)) != 0.0)
        {
          //a non-zero entry will only be created if the corresponding body
          //is affected by gravity(the inverse mass is non-zero)
          if(contact.m_pBody0->isAffectedByGravity())
          {
            VECTOR3 vRj = (dSign0 > Real(0.0)) ? vContacts[j]->m_vPosition0-contact.m_pBody0->com_ : vContacts[j]->m_vPosition1-contact.m_pBody0->com_;
            vTerm0 = contact.m_pBody0->invMass_ * vContacts[j]->m_vNormal;
            vAngularTerm0 =(VECTOR3::Cross(mInvInertiaTensor0 * VECTOR3::Cross(vRj,vContacts[j]->m_vNormal),vR0));
            found=true;
          }
        }

        //check if body 1 is in the j-th contact
        if((dSign1=vContacts[j]->GetSign(contact.m_pBody1)) != 0.0)
        {
          //a non-zero entry will only be created if the corresponding body
          //is affected by gravity(the inverse mass is non-zero)
          if(contact.m_pBody1->isAffectedByGravity())
          {
            VECTOR3 vRj = (dSign1 > Real(0.0)) ? vContacts[j]->m_vPosition0-contact.m_pBody1->com_ : vContacts[j]->m_vPosition1-contact.m_pBody1->com_;
            vTerm1 = ((contact.m_pBody1->invMass_ * vContacts[j]->m_vNormal));
            vAngularTerm1 = (VECTOR3::Cross(mInvInertiaTensor1 * VECTOR3::Cross(vRj,vContacts[j]->m_vNormal),vR1));
            found=true;
          }

        }

        if(found)
        {
          Real val = contact.m_vNormal * (dSign0 * (vTerm0 + vAngularTerm0) - dSign1 * (vTerm1 + vAngularTerm1));
          M.m_dValues[index] = val;
          M.m_iColInd[index] = j;
          index++;
        }

      }//end for j


    }//end for i
#ifdef FC_OPENMP
    printf("time on wall %i = %.3f range: %i \n",omp_get_thread_num(),omp_get_wtime() - wall_timer,i);
}
#endif
}

int CollResponseLcp::ComputeMatrixStructure(std::vector<Contact*> &vContacts, int *rowPointer)
{
  int nContacts = vContacts.size();
  int i,j,entries;
  Real dSign0,dSign1;

  double wall_timer;
  i=0;
  entries = 0;
  rowPointer[0]=0;

  //loop over all contacts
  for(i=0;i<nContacts;i++)
  {
    Contact &contact = *(vContacts[i]);

    int entriesInRow = 0;

    //loop over the row
    for(j=0;j<i;j++)
    {

        bool found = false;
        //assemble off-diagonal
        //check if body 0 is in the j-th contact
        if((dSign0=vContacts[j]->GetSign(contact.m_pBody0)) != 0.0)
        {
          //a non-zero entry will only be created if the corresponding body
          //is affected by gravity(the inverse mass is non-zero)
          if(contact.m_pBody0->isAffectedByGravity())
            found=true;
        }

        //check if body 1 is in the j-th contact
        else if((dSign1=vContacts[j]->GetSign(contact.m_pBody1)) != 0.0)
        {
          //a non-zero entry will only be created if the corresponding body
          //is affected by gravity(the inverse mass is non-zero)
          if(contact.m_pBody1->isAffectedByGravity())
            found=true;
        }

        if(found)
        {
          entries++;
          entriesInRow++;
        }

    }//end for j

    //increase the number of entries in the row
    entriesInRow++;

    //increase the number of entries
    entries++;

    //assemble the remaining elements in the row
    //may have one part, two parts or it can be just 0
    for(j=i+1;j<nContacts;j++)
    {
      bool found = false;
      //assemble off-diagonal
      //check if body 0 is in the j-th contact
      if((dSign0=vContacts[j]->GetSign(contact.m_pBody0)) != 0.0)
      {
        //a non-zero entry will only be created if the corresponding body
        //is affected by gravity(the inverse mass is non-zero)
        if(contact.m_pBody0->isAffectedByGravity())
          found=true;
      }

      //check if body 1 is in the j-th contact
      else if((dSign1=vContacts[j]->GetSign(contact.m_pBody1)) != 0.0)
      {
        //a non-zero entry will only be created if the corresponding body
        //is affected by gravity(the inverse mass is non-zero)
        if(contact.m_pBody1->isAffectedByGravity())
          found=true;
      }

      if(found)
      {
        entries++;
        entriesInRow++;
      }

    }//end for j

    rowPointer[i+1]=rowPointer[i]+entriesInRow;
    //printf("RowPointer[%i]=%i \n",i+1,rowPointer[i+1]);

  }//end for i

  return entries;
}

void CollResponseLcp::ComputeTangentSpace(const VECTOR3& normal, VECTOR3& t1, VECTOR3& t2)
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

void CollResponseLcp::ApplyImpulse(int nContacts, CVectorN<double> &forces, std::vector<Contact*> &vContacts)
{
	
	//calculate responses
	for(int i=0;i<nContacts;i++)
	{
		//Calculate the velocity update
		//get the force
		Real force = (Real)forces(i);

    //save force to the cache
    vContacts[i]->m_dAccumulatedNormalImpulse = force;

    VECTOR3 vR0 = vContacts[i]->m_vPosition0 - vContacts[i]->m_pBody0->com_;
    VECTOR3 vR1 = vContacts[i]->m_vPosition1 - vContacts[i]->m_pBody1->com_;
    VECTOR3 impulse  = vContacts[i]->m_vNormal * force;
    //order of multiplication: this way is more stable if the force is small and the invMass high
    //this may help the solver later on
    VECTOR3 impulse0 =  vContacts[i]->m_vNormal * (force * vContacts[i]->m_pBody0->invMass_);
    VECTOR3 impulse1 = -vContacts[i]->m_vNormal * (force * vContacts[i]->m_pBody1->invMass_);

    //apply the impulse
    vContacts[i]->m_pBody0->applyImpulse(vR0, impulse,impulse0);
    vContacts[i]->m_pBody1->applyImpulse(vR1,-impulse,impulse1);

    //std::cout<<"angular impulse0"<<mInvInertiaTensor0 * (VECTOR3::Cross(vR0,force * vContacts[i].m_vNormal));
    //std::cout<<"angular impulse1"<<mInvInertiaTensor1 * (VECTOR3::Cross(vR1,force * vContacts[i].m_vNormal));
    VECTOR3 t1,t2;
    ComputeTangentSpace(vContacts[i]->m_vNormal, t1, t2);    
    
    std::cout<<"Post-contact  velocity0: "<<vContacts[i]->m_pBody0->velocity_;
    std::cout<<"Post-contact  velocity1: "<<vContacts[i]->m_pBody1->velocity_;
    
    std::cout<<"Normal: "<<vContacts[i]->m_vNormal;
    std::cout<<"t1: "<<t1;
    std::cout<<"t2: "<<t2;
    std::cout<<"Contact point: "<<vContacts[i]->m_vPosition0;        
    //std::cout<<"Post-contact angular velocity0: "<<vContacts[i].m_pBody0->GetAngVel();
    //std::cout<<"Post-contact angular velocity1: "<<vContacts[i].m_pBody1->GetAngVel();
	}

  //std::cout<<"Checking post-contact condition for "<<nContacts<<" colliding contacts."<<std::endl;
	for(int i=0;i<nContacts;i++)
	{
    //check the post-condition of the solver
    VECTOR3 vR0 = vContacts[i]->m_vPosition0 - vContacts[i]->m_pBody0->com_;
    VECTOR3 vR1 = vContacts[i]->m_vPosition1 - vContacts[i]->m_pBody1->com_;
    VECTOR3 relativeVelocity = 
      (vContacts[i]->m_pBody0->velocity_ + (VECTOR3::Cross(vContacts[i]->m_pBody0->getAngVel(),vR0))
     - vContacts[i]->m_pBody1->velocity_ - (VECTOR3::Cross(vContacts[i]->m_pBody1->getAngVel(),vR1)));
    Real relativeNormalVelocity = (relativeVelocity*vContacts[i]->m_vNormal);

    //g_Log.Write("Contact : (%d,%d)",vContacts[i].id0, vContacts[i].id1);
    //g_Log.Write("Post-Contact normal velocity: %lf colliding",relativeNormalVelocity);
    //std::cout<<"Contact between: "<<vContacts[i].id0<<" "<<vContacts[i].id1<<"\n";
    //std::cout<<"Post-Contact normal velocity: "<<relativeNormalVelocity<<"\n";
    //std::cout<<"Post-Contact velocity0: "<<vContacts[i].m_pBody0->m_vVelocity<<"\n";
    //std::cout<<"Post-Contact velocity1: "<<vContacts[i].m_pBody1->m_vVelocity<<"\n";        
    //std::cout<<"Post-contact angular velocity0: "<<vContacts[i].m_pBody0->GetAngVel()<<std::endl;
    //std::cout<<"Post-contact angular velocity1: "<<vContacts[i].m_pBody1->GetAngVel()<<std::endl;
  }

}

}
