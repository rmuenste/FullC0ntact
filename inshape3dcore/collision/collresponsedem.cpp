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

#include "collresponsedem.h"
#include <3dmodel.h>
#include <3dmesh.h>
#include <rigidbody.h>
#include <world.h>
#include <vectorn.h>
#include <perftimer.h>
#include <compoundbody.h>
#include <math.h>
#include <sphere.h>
#include <dembasic.h>
#include <demfriction.h>
#include <demfriction1.h>
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

  CollResponseDEM::CollResponseDEM(void)
  {

  }

  CollResponseDEM::~CollResponseDEM(void)
  {

  }

  CollResponseDEM::CollResponseDEM(std::list<CollisionInfo> *CollInfo, World *pWorld) : CollResponse(CollInfo, pWorld)
  {

  }

  void CollResponseDEM::Solve()
  {

    //return status of our solver
    int ireturnStatus;

    //number of iterations
    int iterations;

    if (this->m_pGraph->edges_->isEmpty())
      return;

    int i, j;
    Real deltaT = m_pWorld->timeControl_->GetDeltaT();

    //number of different contacts
    int nContacts = 0;


    std::vector<RigidBody*> &vRigidBodies = m_pWorld->rigidBodies_;
    std::vector<RigidBody*>::iterator rIter;

    m_iContactPoints = 0;
    std::vector<CollisionInfo*> contacts;
    CollisionHash::iterator hiter = m_pGraph->edges_->begin();
    for (; hiter != m_pGraph->edges_->end(); hiter++)
    {
      CollisionInfo &info = *hiter;
      if (!info.m_vContacts.empty())
      {
        contacts.push_back(&info);
      }
    }

    for (auto &collInfo : contacts)
    {
      Real delta = m_pWorld->timeControl_->GetDeltaT();
      ApplyImpulse(*collInfo, delta);
    }

    for (auto &collInfo : contacts)
    {
      Real delta = m_pWorld->timeControl_->GetDeltaT();
      ApplyStiction(*collInfo, delta);
    }

  }//end Solve

  /**
  * this method uses a simple tangential force model and a linear viscoelastic  normal force model
  */
  void CollResponseDEM::ApplyImpulse(CollisionInfo &ContactInfo, Real &delta)
  {
    std::vector<Contact>::iterator iter;

    //material-dependant constants for normal force (linear viscoelastic model, Fn  = kN*myxi + gammaN*xidot)
    //might better be set as an attribute of the rigid body
    // e.g. brass: kN = 1.79*10^7 (N/m), gammaN = 3.59*10^2(kg/s) 
    //smaller values for kN allow bigger overlaps of the colliding bodies
    Real kN = 1.79E6 / 3.5;  //"spring stiffness" (scaled down to fit particle radius of 0.05 instead of 0.02)
    Real gammaN = 3.59E2; //  == (3.59*10E2) ; //dampening constant (for a velocity-dependant damper)

    //constants for tangential force, both range from 0.0 to 1.0
    Real mu = 0.51;   //coefficient of static friction, 0.51 for brass
    Real gammaT = 0.0025; //static friction strength

    for (iter = ContactInfo.m_vContacts.begin(); iter != ContactInfo.m_vContacts.end(); iter++)
    {
      Contact &contact = *iter;
      //in compounds, the forces and torques acting on each compound are a sum over all forces acting between it's components
      //and any other body in the simulation world. hence, we can apply the forces and torques to the compound for each 
      //contact individually
      if (contact.m_iState != CollisionInfo::TOUCHING)
        continue;

      //compute quantities needed for force model
      //compound- boundarybox, compound-plane, compound-rigidbody, or compound-compound are possible cases for collison

      //compound-compound collision 
      if ((contact.type0 == RigidBody::COMPOUND) && (contact.type1 == RigidBody::COMPOUND))
      {
        DemBasic dem(this->m_pWorld->timeControl_->GetDeltaT());
        dem.evalCompoundCompound(kN, gammaN, mu, gammaT, contact);
        contact.m_iPrevTimeStamp = this->m_pWorld->timeControl_->GetTimeStep();
        contact.m_iTimeStamp = this->m_pWorld->timeControl_->GetTimeStep();
      }
      else if ((contact.type0 == RigidBody::COMPOUND) && (contact.type1 == RigidBody::MESH))
      {
#ifdef DEBUG						
        std::cout << "Collision response compound-mesh" << std::endl;
#endif
        DemBasic dem(this->m_pWorld->timeControl_->GetDeltaT());
        dem.evalCompoundMesh(kN, gammaN, mu, gammaT, contact);
        contact.m_iPrevTimeStamp = this->m_pWorld->timeControl_->GetTimeStep();
        contact.m_iTimeStamp = this->m_pWorld->timeControl_->GetTimeStep();
      }
      else if ((contact.type0 == RigidBody::COMPOUND) && (contact.type1 == RigidBody::BOX))
      {
#ifdef DEBUG
        std::cout << "Collision response compound-mesh" << std::endl;
#endif
        DemBasic dem(this->m_pWorld->timeControl_->GetDeltaT());
        dem.evalCompoundBox(kN, gammaN, mu, gammaT, contact);
        //DemBasic::evalCompoundBox(kN, gammaN, mu, gammaT, contact);
        contact.m_iPrevTimeStamp = this->m_pWorld->timeControl_->GetTimeStep();
        contact.m_iTimeStamp = this->m_pWorld->timeControl_->GetTimeStep();
        //std::cout << "Time: " << this->m_pWorld->timeControl_->GetTime() << " ContactDisplacement: " << contact.contactDisplacement << std::endl;
        //std::cout << "Time: " << this->m_pWorld->timeControl_->GetTime() << " Angvel_V: " << contact.cbody0->getAngVel().y << std::endl;
      }
      //else its a rigid body-rigid body collision
      else
      {
#ifdef DEBUG
        std::cout << "request collider for: " << contact.m_pBody0->shapeId_ << " and " << contact.m_pBody1->shapeId_ << std::endl;
#endif
      }
    }
  }

  /**
  * Apply Stiction force after the other impulses
  */
  void CollResponseDEM::ApplyStiction(CollisionInfo &ContactInfo, Real &delta)
  {
    std::vector<Contact>::iterator iter;

    //material-dependant constants for normal force (linear viscoelastic model, Fn  = kN*myxi + gammaN*xidot)
    //might better be set as an attribute of the rigid body
    // e.g. brass: kN = 1.79*10^7 (N/m), gammaN = 3.59*10^2(kg/s) 
    //smaller values for kN allow bigger overlaps of the colliding bodies
    Real kN = 1.79E6 / 2.5;  //"spring stiffness" (scaled down to fit particle radius of 0.05 instead of 0.02)
    Real gammaN = 3.59E2; //  == (3.59*10E2) ; //dampening constant (for a velocity-dependant damper)

    //constants for tangential force, both range from 0.0 to 1.0
    Real mu = 0.51;   //coefficient of static friction, 0.51 for brass
    Real gammaT = 0.0025; //static friction strength

    for (iter = ContactInfo.m_vContacts.begin(); iter != ContactInfo.m_vContacts.end(); iter++)
    {
      Contact &contact = *iter;
      //in compounds, the forces and torques acting on each compound are a sum over all forces acting between it's components
      //and any other body in the simulation world. hence, we can apply the forces and torques to the compound for each 
      //contact individually
      if (contact.m_iState != CollisionInfo::TOUCHING)
        continue;

      //note that contact.m_pBody0 has the shapeId of the component, and not the whole compound
      if ( ((contact.type0 == RigidBody::COMPOUND) && contact.type1 == RigidBody::BOUNDARYBOX) ||
           ((contact.type0 == RigidBody::COMPOUND) && contact.type1 == RigidBody::HOLLOWCYLINDER) ||
           ((contact.type0 == RigidBody::COMPOUND) && contact.type1 == RigidBody::CYLINDERBDRY))
      {
#ifdef DEBUG						
        std::cout << "Collision response compound-boundarybox" << std::endl;
#endif
        DemBasic dem(this->m_pWorld->timeControl_->GetDeltaT());
        dem.evalCompoundBoundary(kN, gammaN, mu, gammaT, contact);
        contact.m_iPrevTimeStamp = this->m_pWorld->timeControl_->GetTimeStep();
        contact.m_iTimeStamp = this->m_pWorld->timeControl_->GetTimeStep();
      }

    }

  }

}
