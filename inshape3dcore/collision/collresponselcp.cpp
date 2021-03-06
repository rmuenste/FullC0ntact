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

#include <collresponselcp.h>
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

CollResponseLcp::CollResponseLcp(void) : CollResponse(), m_pSolver(nullptr)
{

}

CollResponseLcp::~CollResponseLcp(void)
{
  delete m_pSolver;
}

CollResponseLcp::CollResponseLcp(std::list<CollisionInfo> *CollInfo, World *pWorld) : CollResponse(CollInfo, pWorld)
{
  
}

void CollResponseLcp::InitSolverPGS(int maxIterations, Real omega)
{
  m_iContactPoints = 0;
  dTimeAssembly = 0.0;
  dTimeSolver = 0.0;
  dTimeSolverPost = 0.0;
  dTimeAssemblyDry = 0.0;
  m_pSolver = new LcpSolverGaussSeidel<Real>(maxIterations,omega); 
}

void CollResponseLcp::InitSolverPJA(int maxIterations, Real omega)
{
 m_pSolver = new LcpSolverJacobi<Real>(maxIterations,1.0); 
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
  
  if(this->m_pGraph->edges_->isEmpty())
    return;

  int i,j;
  Real deltaT = this->m_pWorld->timeControl_->GetDeltaT();
  //number of different contacts
  int nContacts=0;
  std::vector<Contact*> vContacts;

  int contactId = 0;
  CollisionHash::iterator hiter = m_pGraph->edges_->begin();
  for(;hiter!=m_pGraph->edges_->end();hiter++)
  {
    CollisionInfo &info = *hiter;
    for (auto &contact : info.m_vContacts)
    {
      if (contact.m_iState == CollisionInfo::TOUCHING)
      {
        contact.contactId_ = contactId++;
        vContacts.push_back(&contact);
      }
    }
  }
  
  nContacts = vContacts.size();
  //std::cout << " number of contacts:  " << nContacts << std::endl;

  if(nContacts == 0)
    return;

  //Initialize matrix and vectors
  //MatrixNxN<Real> M(nContacts,nContacts);
  VectorN<Real> W(nContacts);
  VectorN<Real> Q(nContacts);
  VectorN<Real> Q2(nContacts);    
  VectorN<Real> Z(nContacts);

  std::vector<Contact*>::iterator pIter;
  //get the forces from the contact cache
  for(pIter=vContacts.begin(),i=0;pIter!=vContacts.end();pIter++,i++)
  {
    Contact *contact = *pIter;
    Z(i)=contact->m_dAccumulatedNormalImpulse;
  }

  int *rowPointer = new int[nContacts+1];
  int *rowPointer2 = new int[nContacts+1];
  
//   timer0.Start();
//  int entries2 = computeMatrixStructureGraph(vContacts, rowPointer2);
//   std::cout << "Time graph structure: " << timer0.GetTime() << " entries: " << entries2<< std::endl;
 
  timer0.Start();
  //assemble the matrix
  //int entries = computeMatrixStructure(vContacts,rowPointer);
  int entries2 = computeMatrixStructureGraph(vContacts, rowPointer2);
  dTimeAssemblyDry+=timer0.GetTime();
  //std::cout << "Time normal structure: " << timer0.GetTime() << " entries: " << entries << std::endl;

//   if(entries!=entries2)
//   {
//     std::cout << "number of entries should be the same" << std::endl;
//     for(int k=0;k<nContacts;k++)
//     {
//       int eins;
//       int zwei;
//       eins=rowPointer[k+1]-rowPointer[k];
//       zwei=rowPointer2[k+1]-rowPointer2[k];
//       if(eins!=zwei)
//       {
//         printf("RowPointer[%i]=%i \n",k,eins=rowPointer[k+1]-rowPointer[k]);    
//         printf("RowPointer2[%i]=%i \n",k,zwei=rowPointer2[k+1]-rowPointer2[k]);
//         
//         break;
//       }
//     }
//     exit(1);
//   }

//  MatrixCSR<Real> matrix(vContacts.size(),entries,rowPointer);
  MatrixCSR<Real> matrix2(vContacts.size(),entries2,rowPointer2);

  timer0.Start();
  //assembleVelocityBasedCSR(matrix2,Q,vContacts);
  assembleVelocityBasedCSRGraph(matrix2,Q2,vContacts);
  dTimeAssembly+=timer0.GetTime();

  //timer0.Start();  
  //assembleVelocityBasedCSRGraph(matrix,Q2,vContacts);
  //dTimeAssembly += timer0.GetTime();

  //std::cout << "Time graph assembly: " << timer0.GetTime() << std::endl;
  
  Q2.invert();

  //solve the lcp
  m_pSolver->SetMatrix(matrix2);
  m_pSolver->SetQWZ(Q2,W,Z);
  

  timer0.Start();
  m_pSolver->Solve();
  dTimeSolver+=timer0.GetTime();

  m_iContactPoints = nContacts;

  timer0.Start();
  //apply the impluses calculated by
  //the lcp solver, so after application
  //of the impulses all vn >= 0
  applyImpulse(nContacts,Z,vContacts);
  //std::cout<<"Residual: "<<m_pSolver->m_dResidual<<" Number of iterations used: "<<m_pSolver->m_iIterationsUsed<<"\n";
  //std::cout<<"Number of zero entries: "<<M.NumZeros()<<std::endl;
  m_pSolver->CleanUp();
  dTimeSolverPost+=timer0.GetTime();
  //outputForces(nContacts, Z);
  //std::exit(0);

}//end function

void CollResponseLcp::outputForces(int nContacts, VectorN<Real> &forces)
{

  //calculate responses
  for (int i(0); i < nContacts; ++i)
  {
    std::cout<<"force: ("<<i<< ", "<< forces.m_Data[i] << ")" <<std::endl;
  }

}

void CollResponseLcp::assembleVelocityBased(MatrixNxN<Real> &M, VectorN<Real> &Q, std::vector<Contact*> &vContacts)
{

  std::vector<Contact*>::iterator cIter;
  int nContacts = vContacts.size();
  int i,j;
  Real dSign0,dSign1;
  //loop over all contacts
  //every contact will produce a row in the matrix M
  //for(cIter=vContacts.begin(),i=0;cIter!=vContacts.end();cIter++,i++)
  Real wall_timer;
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

void CollResponseLcp::assembleVelocityBasedCSR(MatrixCSR<Real> &M, VectorN<Real> &Q, std::vector<Contact*> &vContacts)
{
    int nContacts = vContacts.size();
    int i,j,index;
    Real dSign0,dSign1;
    //loop over all contacts
    //every contact will produce a row in the matrix M
    //for(cIter=vContacts.begin(),i=0;cIter!=vContacts.end();cIter++,i++)
    Real wall_timer;
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
      //printf("Contact: index: %i \n",i);

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
            //printf("m[%i]=%f\n",index,M.m_dValues[index]);
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

void CollResponseLcp::assembleVelocityBasedCSRGraph(MatrixCSR<Real> &M, VectorN<Real> &Q, std::vector<Contact*> &vContacts)
{
  int nContacts = vContacts.size();
  int i, j, index;
  Real dSign0, dSign1;
  //loop over all contacts
  //every contact will produce a row in the matrix M
  i = 0;

  for (i = 0; i<nContacts; i++)
  {
    Contact &contact = *(vContacts[i]);

    index = M.m_iRowPtr[i];
    //printf("Thread: %i, row %i index: %i \n",omp_get_thread_num(),i,index);

    //average the restitution
    Real restitution = (contact.m_pBody0->restitution_ * contact.m_pBody1->restitution_);
    VECTOR3 angVel0 = contact.m_pBody0->getAngVel();
    VECTOR3 angVel1 = contact.m_pBody1->getAngVel();

    //get the world-transformed inertia tensor
    MATRIX3X3 mInvInertiaTensor0 = contact.m_pBody0->getWorldTransformedInvTensor();
    MATRIX3X3 mInvInertiaTensor1 = contact.m_pBody1->getWorldTransformedInvTensor();
    VECTOR3 vR0 = contact.m_vPosition0 - contact.m_pBody0->com_;
    VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->com_;
    VECTOR3 vAcc0(0, 0, 0);
    VECTOR3 vAcc1(0, 0, 0);

    VECTOR3 relativeVelocity =
      (contact.m_pBody0->velocity_ + (VECTOR3::Cross(angVel0, vR0))
      - contact.m_pBody1->velocity_ - (VECTOR3::Cross(angVel1, vR1)));

    Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);

    if (!contact.m_pBody0->isAffectedByGravity())
      vAcc0 = VECTOR3(0, 0, 0);
    else
    {
      //gravity + other external acceleration
      vAcc0 = m_pWorld->getGravityEffect(contact.m_pBody0);
      vAcc0 += contact.m_pBody0->force_ * contact.m_pBody0->invMass_;
      vAcc0 += VECTOR3::Cross(contact.m_pBody0->getWorldTransformedInvTensor() * contact.m_pBody0->torque_, vR0);
    }

    if (!contact.m_pBody1->isAffectedByGravity())
      vAcc1 = VECTOR3(0, 0, 0);
    else
    {
      //gravity + other external acceleration
      vAcc1 = m_pWorld->getGravityEffect(contact.m_pBody1);
      vAcc1 += contact.m_pBody1->force_ * contact.m_pBody1->invMass_;
      vAcc1 += VECTOR3::Cross(contact.m_pBody1->getWorldTransformedInvTensor() * contact.m_pBody1->torque_, vR1);
    }

    Q(i) = (1 + restitution) * relativeNormalVelocity + contact.m_vNormal * m_pWorld->timeControl_->GetDeltaT() * (vAcc0 - vAcc1);

    //assemble the diagonal element
    //the diagonal element of a contact has always two parts,
    //one for the first body and one for the second
    //only the point of application on the body
    //and the direction of the contact normal differ
    Real term0 = contact.m_pBody0->invMass_;
    Real angularTerm0 = contact.m_vNormal * ((VECTOR3::Cross(mInvInertiaTensor0 * VECTOR3::Cross(vR0, contact.m_vNormal), vR0)));

    Real term1 = contact.m_pBody1->invMass_;
    Real angularTerm1 = contact.m_vNormal * ((VECTOR3::Cross(mInvInertiaTensor1 * VECTOR3::Cross(vR1, contact.m_vNormal), vR1)));

    //on the diagonal we add the terms
    //that means the diagonal element is N_i * [(m_a^-1*N_i + N_i * (J_a^-1*(r_ia x N_i)) x r_ia) + (m_b^-1*N_i + N_i * (J_b^-1*(r_ib x N_i)) x r_ib)]
    M.m_dValues[index] = term0 + angularTerm0 + term1 + angularTerm1;
    M.m_iColInd[index] = i;

    //increase the index into the entries array
    index++;

    for (auto j : rowStructure[i])
    {
      
      if(j==i)
        continue;
      
      //initialize the entry with zero
      //the entry is non-zero only in case the
      //jth-contact includes the body0 or body1 of the
      //ith-contact
      bool found = false;
      VECTOR3 vTerm0 = VECTOR3(0, 0, 0);
      VECTOR3 vAngularTerm0 = VECTOR3(0, 0, 0);
      VECTOR3 vTerm1 = VECTOR3(0, 0, 0);
      VECTOR3 vAngularTerm1 = VECTOR3(0, 0, 0);

      //assemble off-diagonal
      //check if body 0 is in the j-th contact
      if ((dSign0 = vContacts[j]->GetSign(contact.m_pBody0)) != 0.0)
      {
        //a non-zero entry will only be created if the corresponding body
        //is affected by gravity(the inverse mass is non-zero)
        if (contact.m_pBody0->isAffectedByGravity())
        {
          VECTOR3 vRj = (dSign0 > Real(0.0)) ? vContacts[j]->m_vPosition0 - contact.m_pBody0->com_ : vContacts[j]->m_vPosition1 - contact.m_pBody0->com_;
          //VECTOR3 vRj = (dSign0 > Real(0.0)) ? vContacts[j].m_vPosition0 : vContacts[j].m_vPosition1;
          vTerm0 = contact.m_pBody0->invMass_ * vContacts[j]->m_vNormal;
          vAngularTerm0 = (VECTOR3::Cross(mInvInertiaTensor0 * VECTOR3::Cross(vRj, vContacts[j]->m_vNormal), vR0));
          found = true;
        }
      }
      //check if body 1 is in the j-th contact
      if ((dSign1 = vContacts[j]->GetSign(contact.m_pBody1)) != 0.0)
      {
        //a non-zero entry will only be created if the corresponding body
        //is affected by gravity(the inverse mass is non-zero)
        if (contact.m_pBody1->isAffectedByGravity())
        {
          found = true;
          VECTOR3 vRj = (dSign1 > Real(0.0)) ? vContacts[j]->m_vPosition0 - contact.m_pBody1->com_ : vContacts[j]->m_vPosition1 - contact.m_pBody1->com_;
          //VECTOR3 vRj = (dSign1 > Real(0.0)) ? vContacts[j].m_vPosition0 : vContacts[j].m_vPosition1;
          vTerm1 = ((contact.m_pBody1->invMass_ * vContacts[j]->m_vNormal));
          vAngularTerm1 = (VECTOR3::Cross(mInvInertiaTensor1 * VECTOR3::Cross(vRj, vContacts[j]->m_vNormal), vR1));
        }

      }

      if (found)
      {
        Real val = contact.m_vNormal * (dSign0 * (vTerm0 + vAngularTerm0) - dSign1 * (vTerm1 + vAngularTerm1));
        M.m_dValues[index] = val;
        M.m_iColInd[index] = j;
        index++;
      }

    }

  }//end for i

  delete[] rowStructure;

}

int CollResponseLcp::computeMatrixStructureGraph(std::vector<Contact*> &vContacts, int *rowPointer)
{
  int nContacts = vContacts.size();
  int i, entries;
  Real dSign0, dSign1;

  Real wall_timer;
  i = 0;
  entries = 0;
  rowPointer[0] = 0;

  rowStructure = new std::list<int>[nContacts];
  //loop over all contacts
  for (i = 0; i<nContacts; i++)
  {
    Contact &contact = *(vContacts[i]);
    std::set<int> rowEntries;
    int entriesInRow = 0;

    RigidBody *b0 = contact.m_pBody0;
    RigidBody *b1 = contact.m_pBody1;

    if(b0->isAffectedByGravity())
    {
    for (auto &k : b0->getEdges())
    {
      CollisionInfo *info = k;
      if (info->m_iState == CollisionInfo::TOUCHING || info->m_iState == CollisionInfo::PERSISTENT_TOUCHING)
      {
        for (auto &l : info->m_vContacts)
        {
          if (l.m_iState == CollisionInfo::TOUCHING)
          {
            if(l.contactId_<0)
            {
              exit(1);
            }
            rowEntries.insert(l.contactId_);
          }
        }
      }
    }
    }

    if(b1->isAffectedByGravity())
    {    
    for (auto &k : b1->getEdges())
    {
      CollisionInfo *info = k;
      if (info->m_iState == CollisionInfo::TOUCHING || info->m_iState == CollisionInfo::PERSISTENT_TOUCHING)
      {
        for (auto &l : info->m_vContacts)
        {
          if (l.m_iState == CollisionInfo::TOUCHING)
          {
            if(l.contactId_<0)
            {
              exit(1);
            }            
            rowEntries.insert(l.contactId_);
          }
        }
      }
    }
    }

    entriesInRow = rowEntries.size();
    entries += rowEntries.size();
    rowPointer[i + 1] = rowPointer[i] + entriesInRow;

    
    for (auto j : rowEntries)
    {
      rowStructure[i].push_back(j);
    }

    //printf("RowPointer[%i]=%i \n",i+1,rowPointer[i+1]);

  }//end for i
  
  return entries;
}

int CollResponseLcp::computeMatrixStructure(std::vector<Contact*> &vContacts, int *rowPointer)
{
  int nContacts = vContacts.size();
  int i,j,entries;
  Real dSign0,dSign1;

  Real wall_timer;
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

void CollResponseLcp::computeTangentSpace(const VECTOR3& normal, VECTOR3& t1, VECTOR3& t2)
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

void CollResponseLcp::applyImpulse(int nContacts, VectorN<Real> &forces, std::vector<Contact*> &vContacts)
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
    computeTangentSpace(vContacts[i]->m_vNormal, t1, t2);
    //
    //std::cout<< "force: "<< force << "Post-contact  velocity0: "<<vContacts[i]->m_pBody0->velocity_;
    //std::cout<<"Post-contact  velocity1: "<<vContacts[i]->m_pBody1->velocity_;

    //std::cout<<"Normal: "<<vContacts[i]->m_vNormal;
    //std::cout<<"t1: "<<t1;
    //std::cout<<"t2: "<<t2;
    //std::cout<<"Contact point: "<<vContacts[i]->m_vPosition0;
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

//    VECTOR3 t0;
//    VECTOR3 t1;
//    computeTangentSpace(vContacts[i]->m_vNormal, t0, t1);
//
//    std::cout<<"--------------------TangentSpace--------------------"<<std::endl;
//    std::cout<<"Contact point: "<<vContacts[i]->m_vPosition0;
//    std::cout<<"Normal: "<<vContacts[i]->m_vNormal;
//    std::cout<<"t0: "<<t0;
//    std::cout<<"t1: "<<t1;
//    int nu = 6;
//    for(int i=0; i<nu; i++)
//    {
//      VECTOR3 dhk(cos(2.0 * (Real(i)-1.0)*CMath<Real>::SYS_PI/Real(nu))*t0 + sin(2.0*(Real(i)-1.0)*CMath<Real>::SYS_PI/Real(nu))*t1);
//      printf("d_%i(%f,%f,%f) \n",i,dhk.x,dhk.y,dhk.z);
//    }
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
