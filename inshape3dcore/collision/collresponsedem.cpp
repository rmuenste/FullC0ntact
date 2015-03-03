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

CollResponseDEM::CollResponseDEM(std::list<CollisionInfo> *CollInfo, World *pWorld) : CollResponse(CollInfo,pWorld)
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
		//for (auto &c : collInfo->m_vContacts)
		//{
		//	int id = c.subId0;
		//}
		Real delta = m_pWorld->timeControl_->GetDeltaT();
		ApplyImpulse(*collInfo, delta);
	}


}//end Solve



/**
this method uses a simple tangential force model and a linear viscoelastic  normal force model */
void CollResponseDEM::ApplyImpulse(CollisionInfo &ContactInfo, Real &delta)
{
	std::vector<Contact>::iterator iter;

	//material-dependant constants for normal force (linear viscoelastic model, Fn  = kN*myxi + gammaN*xidot)
	//might better be set as an attribute of the rigid body
	// e.g. brass: kN = 1.79*10^7 (N/m), gammaN = 3.59*10^2(kg/s) 
	//smaller values for kN allow bigger overlaps of the colliding bodies
	Real kN = 1.79E6/2.5;  //"spring stiffness" (scaled down to fit particle radius of 0.05 instead of 0.02)
	Real gammaN = 3.59E2; //  == (3.59*10E2) ; //dampening constant (for a velocity-dependant damper)

	//constants for tangential force, both range from 0.0 to 1.0
	Real mu =  0.51;   //coefficient of static friction, 0.51 for brass
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
		
		//note that contact.m_pBody0 has the shapeId of the component, and not the whole compound
			if ((contact.type0 == RigidBody::COMPOUND) && contact.type1 == RigidBody::BOUNDARYBOX ){
#ifdef DEBUG						
        std::cout<<"Collision response compound-boundarybox"<<std::endl;
#endif
				RigidBody *subbody = contact.cbody0->rigidBodies_[contact.subId0];

				//radius of the component sphere 
				Real R0 = subbody->shape_->getAABB().extents_[0];
				//radius of the second body (only if it is not a compound)

         //compound - boundarybox case; compound-plane is similar to this case 
				Real xjxq = contact.m_dDistance;

				Real myxi = std::max(R0 - xjxq, 0.0);
				
				//relative velocity of the contact point
				Real xidot = -((subbody->velocity_) * (-contact.m_vNormal));

				//compute normal force using linear viscoelastic model
				Real Fn = kN*myxi  + gammaN*xidot;

				//making sure Fn has a non-negative value;
				if (Fn < 1.0E-6 || myxi < 1.0E-12)
        {
					Fn = 0.0;
				}

#ifdef DEBUG
        std::cout <<"simtime: " << m_pWorld->timeControl_->GetTime() << " overlap: " << myxi << std::endl;
        std::cout << "kN*overlap: " << kN*myxi << " dampening: " << gammaN*xidot << std::endl;
        std::cout << "Normal force: " << Fn << std::endl;
        std::cout << "overlap: " << R0 - xjxq << std::endl;
#endif
				VECTOR3 normalImpulse = Fn * contact.m_vNormal;

				//to compute tangential force, the relative velocity of the contact point  in regard to the whole bodies is needed 
				//the relative positions of the contact point on each body
				VECTOR3 z = subbody->getTransformedPosition() + (R0 - myxi / 2.0) * (-contact.m_vNormal);
				//VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->com_;

				VECTOR3 relAngVel = VECTOR3::Cross(-contact.cbody0->getAngVel(), z - contact.cbody0->com_);
				VECTOR3 relVel = (-subbody->velocity_) + relAngVel;

        MATRIX3X3 rot = contact.cbody0->getQuaternion().GetMatrix();
        MATRIX3X3 w2l = contact.cbody0->getQuaternion().GetMatrix();
        w2l.TransposeMatrix();

        VECTOR3 omega_world = rot * contact.cbody0->getAngVel();

        VECTOR3 relAngVelt = VECTOR3::Cross(-omega_world, z - contact.cbody0->com_);

        VECTOR3 relVelt = (-subbody->velocity_) + relAngVelt;


				VECTOR3 tangentVel = relVel - (relVel * contact.m_vNormal * contact.m_vNormal);
				VECTOR3 tangentImpulse = tangentVel;

        VECTOR3 tangentVel_t = relVelt - (relVelt * contact.m_vNormal * contact.m_vNormal);
        VECTOR3 tangentImpulse_t = tangentVel_t;
#ifdef DEBUG						
        std::cout << "world omega: " << omega_world;
        std::cout << "local omega: " << contact.cbody0->getAngVel();
        std::cout << "world2local omega: " << w2l*omega_world;
        std::cout << "normal: " << contact.m_vNormal;
        std::cout << "contact point: " << z;

        std::cout << "tangentVel: " << tangentVel;
        std::cout << "RelVel_t: " << relVelt << std::endl;
        std::cout << "RelVel: " << relVel << std::endl;

        std::cout << "RelVel_tn: " << relVelt*-contact.m_vNormal << std::endl;
        std::cout << "RelVeln: " << relVel*-contact.m_vNormal << std::endl;
#endif
				Real Ft1 = mu * normalImpulse.mag();
				Real Ft2 = gammaT * tangentVel.mag();



				//tangential force is limited by coloumb`'s law of frictrion
				Real min = -(std::min(Ft1, Ft2));
				
				//normalize the vector
				if(tangentVel.mag() != 0.0)
        {
					tangentImpulse = -1.0* tangentVel * (min / tangentVel.mag());
				}
        //normalize the vector
        if (tangentVel_t.mag() != 0.0)
        {
          tangentImpulse_t = -1.0* tangentVel_t * (min / tangentVel_t.mag());
        }

				//compute the torques for the compound body
				VECTOR3 Torque0 = VECTOR3(0.0, 0.0, 0.0);

        //compute the torques for the compound body
        VECTOR3 Torque0_t = VECTOR3(0.0, 0.0, 0.0);
				
				//and the force; they are only applied if there is an overlap, i.e. if myxi >0
				VECTOR3 Force0 = VECTOR3(0.0, 0.0, 0.0);
				
				if (myxi > 1.0E-6)
        {
          Force0 = (normalImpulse + tangentImpulse) * contact.cbody0->invMass_;
          //normal force may only be applied while relative normal velocity of the contact point 
          // (relVel*n) is negative
          if (relVel*(-contact.m_vNormal) > 1.0E-6)
	          Force0 = VECTOR3(0.0, 0.0, 0.0);

          VECTOR3 vCP = z - contact.cbody0->com_;
#ifdef DEBUG						
          std::cout << "rCP: " << z - contact.cbody0->com_;
#endif

          Torque0 = VECTOR3::Cross(z - contact.cbody0->com_, tangentImpulse);
          Torque0_t = VECTOR3::Cross(z - contact.cbody0->com_, tangentImpulse_t);

          VECTOR3 vCross;

          vCross.x = ((vCP.y * tangentImpulse.z) - (vCP.z * tangentImpulse.y));

          vCross.y = ((vCP.z * tangentImpulse.x) - (vCP.x * tangentImpulse.z));

          vCross.z = ((vCP.x * tangentImpulse.y) - (vCP.y * tangentImpulse.x));
#ifdef DEBUG						
          std::cout << "Cross: " << vCross;
#endif
				}

#ifdef DEBUG						
        std::cout << "tangential impule: " << tangentImpulse;
        std::cout << "tangential velocity: " << tangentVel;

        std::cout << "tangential impule_t: " << tangentImpulse_t;
        std::cout << "tangential velocity_t: " << tangentVel_t;

        std::cout << "Torque: " << Torque0;
        std::cout << "Torque_t: " << Torque0_t;
#endif


        //for motionintegratorDEM based on taylor expansion, the applied forces for each component of a compound
        //are stored in the variables ComponentForces_ and ComponentTorques_ respectively.
        //these are then applied together with gravity within one timestep in the motionintegrator
		    contact.cbody0->force_ += Force0;
			  contact.cbody0->torque_ += Torque0;

        contact.cbody0->force_local_ += Force0;
        contact.cbody0->torque_local_ += Torque0_t;
			 
				//now apply forces and torques
				//contact.cbody0->applyForces(Force0, Torque0, delta);
        //std::cout<<"velocity after: "<< contact.cbody0->velocity_ <<std::endl;
			
			  //and to the boundary box 
				//contact.m_pBody1->applyForces(Force1, Torque1);

			}

	     //compound-compound collision 
			 else if ((contact.type0 == RigidBody::COMPOUND) && (contact.type1 == RigidBody::COMPOUND)){

#ifdef DEBUG						
	      std::cout<<"Collision response compound-compound"<<std::endl;
#endif
				RigidBody *subbody0 = contact.cbody0->rigidBodies_[contact.subId0];
				RigidBody *subbody1 = contact.cbody1->rigidBodies_[contact.subId1];

				//radius of the component sphere 
				Spherer *sphere = dynamic_cast<Spherer *>(subbody0->shape_);
				Real R0 = sphere->getRadius();
				Real R1 = sphere->getRadius();

				//compute xi 
				Real xjxq = contact.m_dDistance;
				Real xi = std::max(R0 + R1 - xjxq, 0.0);
				Real ovr = xi;

				//contact.m_vNormal should have the correct sign, so we can obtain it directly from contact

				//compute xidot
				Real xidot = (subbody0->velocity_ - subbody1->velocity_) * (- contact.m_vNormal);

				//the contact point
				VECTOR3 ztest = subbody0->getTransformedPosition();
				VECTOR3 ztest2 = ((R0 - xi / 2.0) * (-contact.m_vNormal));
				VECTOR3 z = subbody0->getTransformedPosition() + ((R0 - xi / 2.0) * (- contact.m_vNormal));

        MATRIX3X3 rot0 = contact.cbody0->getQuaternion().GetMatrix();
        MATRIX3X3 rot1 = contact.cbody1->getQuaternion().GetMatrix();

        VECTOR3 omega_world0 = rot0 * contact.cbody0->getAngVel();
        VECTOR3 omega_world1 = rot1 * contact.cbody1->getAngVel();

        //velocities of the contact points relative to the whole compounds
        VECTOR3 relAngVel_w = VECTOR3::Cross(omega_world1, z - contact.cbody1->com_)
                            - VECTOR3::Cross(omega_world0, z - contact.cbody0->com_);

        VECTOR3 relVel_w = contact.cbody1->velocity_ - contact.cbody0->velocity_ + relAngVel_w;


				//velocities of the contact points relative to the whole compounds
				VECTOR3 relAngVel = VECTOR3::Cross(contact.cbody1->getAngVel(), z - contact.cbody1->com_) 
					                - VECTOR3::Cross(contact.cbody0->getAngVel(), z - contact.cbody0->com_);

				VECTOR3 relVel = contact.cbody1->velocity_ - contact.cbody0->velocity_ + relAngVel;

				//normal force, linear viscoelastic model 
				Real Fn = kN*xi + gammaN*xidot;

				//making sure Fn has a non-negative value 
				if (Fn < 1.0E-6 || xi < 1.0E-12){
					Fn = 0.0;
				}
				VECTOR3 normalImpulse = -(Fn * (-contact.m_vNormal));
#ifdef DEBUG						
        std::cout<<"Particle-Particle: kN*overlap: "<< kN*xi << " dampening: " << gammaN*xidot <<std::endl;
#endif
				//tangential force 

        VECTOR3 tangentVel = relVel_w - (relVel_w * (-contact.m_vNormal) * (-contact.m_vNormal));
				VECTOR3 tangentImpulse = tangentVel;

				Real Ft1 = mu * normalImpulse.mag();
				Real Ft2 = gammaT * tangentVel.mag();

				//tangential force is limited by coloumb`'s law of frictrion
				Real min = -(std::min(Ft1, Ft2));

				//normalize the vector
				if (tangentVel.mag() != 0.0)
        {
					tangentImpulse = tangentVel * (min / tangentVel.mag());
				}
        //std::cout<<"Particle-Particle: tangential impulse" << gammaN*xidot <<std::endl;
				//compute the torques for the compound body
				VECTOR3 Torque0 = VECTOR3(0.0, 0.0, 0.0);
				VECTOR3 Torque1 = VECTOR3(0.0, 0.0, 0.0);

				//and the force; they are only applied if there is an overlap, i.e. if myxi >0
				VECTOR3 Force0 = VECTOR3(0.0, 0.0, 0.0);
				VECTOR3 Force1 = VECTOR3(0.0, 0.0, 0.0);

				if (xi > 1.0E-6){
					Force0 = (normalImpulse + tangentImpulse) * contact.cbody0->invMass_;
					Force1 = -(normalImpulse + tangentImpulse) * contact.cbody1->invMass_;

					//normal force may only be applied while relative normal velocity of the contact point 
					// (relVel*n) is negative
          Torque0 = VECTOR3::Cross(z - contact.cbody0->com_, tangentImpulse);
          Torque0 = -1.0 * Torque0;
          Torque1 = VECTOR3::Cross(z - contact.cbody1->com_, tangentImpulse);

          if ((relVel_w*(-contact.m_vNormal) > 1.0E-6) && (2.0*R0 - xi) < 0.025*R0)
					{
						Force0 = VECTOR3(0.0, 0.0, 0.0);
					  Force1 = VECTOR3(0.0, 0.0, 0.0);
            Torque0 = VECTOR3(0.0, 0.0, 0.0);
            Torque1 = VECTOR3(0.0, 0.0, 0.0);
					}


				}

#ifdef DEBUG
				std::cout<< "simulation_time: " << m_pWorld->timeControl_->GetTime() << " overlap: " << xi <<std::endl;
#endif

        //for motionintegratorDEM based on taylor expansion, the applied forces for each component of a compound
				//are added onto the variables force_ and torque_ since the force and torque acting on a compound 
				//is the sum of the forces and torques of each component
				//these are then applied together with gravity within one timestep in the motionintegrator
				contact.cbody0->force_ += Force0;
				contact.cbody0->torque_local_ += Torque0;
				contact.cbody1->force_ += Force1;
				contact.cbody1->torque_local_ += Torque1;


				//now apply forces and torques
				//contact.cbody0->applyForces(Force0, Torque0, delta);
				//contact.cbody1->applyForces(Force1, Torque1, delta);
				
			}
			else if ((contact.type0 == RigidBody::COMPOUND) && (contact.type1 == RigidBody::MESH))
			{
#ifdef DEBUG						
        std::cout<<"Collision response compound-mesh"<<std::endl;
#endif
        //The normal points from the compound sphere to the mesh object
        contact.m_vNormal=-contact.m_vNormal;

        RigidBody *subbody = contact.cbody0->rigidBodies_[contact.subId0];

        //radius of the component sphere
        Real R0 = subbody->shape_->getAABB().extents_[0];
        //radius of the second body (only if it is not a compound)
        //Real R1 = 0;

         //compound - boundarybox case; compound-plane is similar to this case
        Real xjxq = contact.m_dDistance;

        // VECTOR3 ztest = subbody->getTransformedPosition();

        Real myxi = std::max(R0 - xjxq, 0.0);

        //relative velocity of the contact point
        Real xidot = subbody->velocity_ * contact.m_vNormal;

        //compute normal force using linear viscoelastic model
        Real Fn = kN*myxi  + gammaN*xidot;

#ifdef DEBUG						
        std::cout<<"kN*overlap: "<< kN*myxi << " dampening: " << gammaN*xidot <<std::endl;
        std::cout<<"overlap: "<< R0-xjxq << std::endl;
#endif

        //making sure Fn has a non-negative value;
        if (Fn < 1.0E-6 || myxi < 1.0E-12){
          Fn = 0.0;
        }

        VECTOR3 normalImpulse = Fn * contact.m_vNormal;

        //to compute tangential force, the relative velocity of the contact point  in regard to the whole bodies is needed
        //the relative positions of the contact point on each body
        VECTOR3 z = subbody->getTransformedPosition() + (R0 - myxi / 2.0) * (-contact.m_vNormal);
        //z = contact.m_vPosition0;
        //VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->com_;

        MATRIX3X3 rot = contact.cbody0->getQuaternion().GetMatrix();
        MATRIX3X3 w2l = contact.cbody0->getQuaternion().GetMatrix();
        w2l.TransposeMatrix();

        VECTOR3 omega_world = rot * contact.cbody0->getAngVel();

        VECTOR3 relAngVel_world = VECTOR3::Cross(-omega_world, z - contact.cbody0->com_);

        VECTOR3 relVel_world = (-subbody->velocity_) + relAngVel_world;

        VECTOR3 tangentVel_t = relVel_world - (relVel_world * (contact.m_vNormal) * (contact.m_vNormal));
        VECTOR3 tangentImpulse_t = tangentVel_t;

#ifdef DEBUG						
        std::cout << "world omega: " << omega_world;
        std::cout << "local omega: " << contact.cbody0->getAngVel();
        std::cout << "world2local omega: " << w2l*omega_world;

        std::cout << "tangentVel: " << tangentVel;
        std::cout << "RelVel_t: " << relVelt*-contact.m_vNormal << std::endl;
#endif

        Real Ft1 = mu * normalImpulse.mag();
        Real Ft2 = gammaT * tangentVel_t.mag();

        //tangential force is limited by coloumb`'s law of frictrion
        Real min = -(std::min(Ft1, Ft2));

        //normalize the vector
        if (tangentVel_t.mag() != 0.0)
        {
          tangentImpulse_t = -1.0* tangentVel_t * (min / tangentVel_t.mag());
        }

#ifdef DEBUG						
        std::cout << "tangentVel: " << tangentVel;
        std::cout << "tangentImpulse: " << tangentImpulse;

        std::cout << "tangentVel_world: " << tangentVel_t;
        std::cout << "tangentImpulse_world: " << tangentImpulse_t;
#endif

        //compute the torques for the compound body
        VECTOR3 Torque0 = VECTOR3(0.0, 0.0, 0.0);
        VECTOR3 Torque0_t = VECTOR3(0.0, 0.0, 0.0);

        //and the force; they are only applied if there is an overlap, i.e. if myxi >0
        VECTOR3 Force0 = VECTOR3(0.0, 0.0, 0.0);

#ifdef DEBUG						
        std::cout << "RelVel: " << relVel*-contact.m_vNormal << std::endl;
        std::cout << "RelVel_t: " << relVelt*-contact.m_vNormal << std::endl;
        std::cout << "RelVel_transl: " << (-subbody->velocity_)*-contact.m_vNormal << std::endl;
        std::cout << "RelVel_rot: " << (VECTOR3::Cross(-omega_world, z - contact.cbody0->com_))*-contact.m_vNormal << std::endl;
        
        std::cout << "contact point: " << z;
#endif
        //VECTOR3 z_t = rot * (z - contact.cbody0->com_);
        Torque0_t = VECTOR3::Cross(z - contact.cbody0->com_, tangentImpulse_t);

        if (xjxq <= R0)
        {
          Force0 = (normalImpulse + tangentImpulse_t) * contact.cbody0->invMass_;
          //normal force may only be applied while relative normal velocity of the contact point
          // (relVel*n) is negative

          if (relVel_world*(-contact.m_vNormal) > 1.0E-6)// && (R0 - xjxq) < 0.025*R0)
          {
            Force0 = VECTOR3(0.0, 0.0, 0.0);
            Torque0 = VECTOR3(0.0, 0.0, 0.0);
            Torque0_t = VECTOR3(0.0, 0.0, 0.0);
          }
        }

#ifdef DEBUG						
        //        std::cout<<"Overlap: "<< myxi <<std::endl;
        std::cout<<"normal Force: "<< Force0 <<std::endl;
        //        std::cout<<"velocity before: "<< contact.cbody0->velocity_ <<std::endl;
        std::cout<<"Torque0_mix "<< Torque0 <<std::endl;
        std::cout << "Torque0_world " << Torque0_t << std::endl;
        std::cout << "Torque0_trans " << w2l*Torque0 << std::endl;
#endif

        //for motionintegratorDEM based on taylor expansion, the applied forces for each component of a compound
        //are stored in the variables ComponentForces_ and ComponentTorques_ respectively.
        //these are then applied together with gravity within one timestep in the motionintegrator
        contact.cbody0->force_ += Force0;
        contact.cbody0->torque_ += Torque0_t;

        contact.cbody0->force_local_  += Force0;
        contact.cbody0->torque_local_ += Torque0_t;

    }
    else if ((contact.type0 == RigidBody::COMPOUND) && (contact.type1 == RigidBody::BOX))
    {

#ifdef DEBUG
        std::cout<<"Collision response compound-mesh"<<std::endl;
#endif
        //Normal points from the box to the compound
        contact.m_vNormal = -contact.m_vNormal;

        RigidBody *subbody = contact.cbody0->rigidBodies_[contact.subId0];

        //radius of the component sphere
        Real R0 = subbody->shape_->getAABB().extents_[0];
        //radius of the second body (only if it is not a compound)
        //Real R1 = 0;

         //compound - boundarybox case; compound-plane is similar to this case
        Real xjxq = contact.m_dDistance;

        Real myxi = std::max(R0 - xjxq, 0.0);

        //relative velocity of the contact point
        Real xidot = (subbody->velocity_- contact.m_pBody0->velocity_) * (-contact.m_vNormal);

        //compute normal force using linear viscoelastic model
        Real Fn = kN*myxi  + gammaN*xidot;

#ifdef DEBUG
        std::cout<<"kN*overlap: "<< kN*myxi << " dampening: " << gammaN*xidot <<std::endl;
        std::cout<<"overlap: "<< R0-xjxq << std::endl;
#endif

        //making sure Fn has a non-negative value;
        if (Fn < 1.0E-6 || myxi < 1.0E-12){
          Fn = 0.0;
        }

        VECTOR3 normalImpulse = Fn * contact.m_vNormal;

        //to compute tangential force, the relative velocity of the contact point  in regard to the whole bodies is needed
        //the relative positions of the contact point on each body
        VECTOR3 z = subbody->getTransformedPosition() + (R0 - myxi / 2.0) * (contact.m_vNormal);
        //z = contact.m_vPosition0;
        //VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->com_;

        MATRIX3X3 rot = contact.cbody0->getQuaternion().GetMatrix();
        MATRIX3X3 w2l = contact.cbody0->getQuaternion().GetMatrix();
        w2l.TransposeMatrix();

        VECTOR3 omega_world = rot * contact.cbody0->getAngVel();

        VECTOR3 relAngVelt = VECTOR3::Cross(-omega_world, z - contact.cbody0->com_);

        VECTOR3 relVelt = (-subbody->velocity_) + relAngVelt;

        VECTOR3 relAngVel = VECTOR3::Cross(-contact.cbody0->getAngVel(), z - contact.cbody0->com_);
        VECTOR3 relVel = (-subbody->velocity_) + relAngVel;

        VECTOR3 tangentVel_t = relVelt - (relVelt * (contact.m_vNormal) * (contact.m_vNormal));
        VECTOR3 tangentImpulse_t = tangentVel_t;

#ifdef DEBUG
        std::cout << "world omega: " << omega_world;
        std::cout << "local omega: " << contact.cbody0->getAngVel();
        std::cout << "world2local omega: " << w2l*omega_world;

        std::cout << "tangentVel: " << tangentVel;
        std::cout << "RelVel_t: " << relVelt*-contact.m_vNormal << std::endl;
#endif

//        Real Ft1 = mu * normalImpulse.mag();
//        Real Ft2 = gammaT * tangentVel.mag();
//
//        //tangential force is limited by coloumb`'s law of frictrion
//        Real min = -(std::min(Ft1, Ft2));
//
//        //normalize the vector
//        if (tangentVel.mag() != 0.0)
//        {
//          tangentImpulse = -1.0* tangentVel * (min / tangentVel.mag());
//        }
//
//        if (tangentVel_t.mag() != 0.0)
//        {
//          tangentImpulse_t = -1.0* tangentVel_t * (min / tangentVel_t.mag());
//        }

#ifdef DEBUG
        std::cout << "tangentVel: " << tangentVel;
        std::cout << "tangentImpulse: " << tangentImpulse;

        std::cout << "tangentVel_world: " << tangentVel_t;
        std::cout << "tangentImpulse_world: " << tangentImpulse_t;
#endif

        //compute the torques for the compound body
        VECTOR3 Torque0 = VECTOR3(0.0, 0.0, 0.0);
        VECTOR3 Torque0_t = VECTOR3(0.0, 0.0, 0.0);

        //and the force; they are only applied if there is an overlap, i.e. if myxi >0
        VECTOR3 Force0 = VECTOR3(0.0, 0.0, 0.0);

#ifdef DEBUG
        std::cout << "RelVel: " << relVel*-contact.m_vNormal << std::endl;
        std::cout << "RelVel_t: " << relVelt*-contact.m_vNormal << std::endl;
        std::cout << "RelVel_transl: " << (-subbody->velocity_)*-contact.m_vNormal << std::endl;
        std::cout << "RelVel_rot: " << (VECTOR3::Cross(-omega_world, z - contact.cbody0->com_))*-contact.m_vNormal << std::endl;

        std::cout << "contact point: " << z;
#endif
//        Torque0 = VECTOR3::Cross(z - contact.cbody0->com_, tangentImpulse);
//        //VECTOR3 z_t = rot * (z - contact.cbody0->com_);
//        Torque0_t = VECTOR3::Cross(z - contact.cbody0->com_, tangentImpulse_t);

        if (xjxq <= R0)
        {
          //Force0 = (normalImpulse + tangentImpulse) * contact.cbody0->invMass_;
          Force0 = (normalImpulse) * contact.cbody0->invMass_;
          //normal force may only be applied while relative normal velocity of the contact point
          // (relVel*n) is negative

          if (-xidot > 1.0E-6)// && (R0 - xjxq) < 0.025*R0)
          {
            Force0 = VECTOR3(0.0, 0.0, 0.0);
            Torque0 = VECTOR3(0.0, 0.0, 0.0);
            Torque0_t = VECTOR3(0.0, 0.0, 0.0);
          }



        }

#ifdef DEBUG
        //        std::cout<<"Overlap: "<< myxi <<std::endl;
        std::cout<<"normal Force: "<< Force0 <<std::endl;
        //        std::cout<<"velocity before: "<< contact.cbody0->velocity_ <<std::endl;
        std::cout<<"Torque0_mix "<< Torque0 <<std::endl;
        std::cout << "Torque0_world " << Torque0_t << std::endl;
        std::cout << "Torque0_trans " << w2l*Torque0 << std::endl;
#endif


        //for motionintegratorDEM based on taylor expansion, the applied forces for each component of a compound
        //are stored in the variables ComponentForces_ and ComponentTorques_ respectively.
        //these are then applied together with gravity within one timestep in the motionintegrator
        contact.cbody0->force_ += Force0;
        contact.cbody0->torque_ += Torque0_t;

        contact.cbody0->force_local_  += Force0;
        contact.cbody0->torque_local_ += Torque0_t;

        //now apply forces and torques
        //contact.cbody0->applyForces(Force0, Torque0, delta);
        //std::cout<<"velocity after: "<< contact.cbody0->velocity_ <<std::endl;

        //and to the boundary box
        //contact.m_pBody1->applyForces(Force1, Torque1);

    }
    else if ((contact.type0 == RigidBody::COMPOUND) && (contact.type1 == RigidBody::BOX))
    {
        #ifdef DEBUG
                std::cout<<"Collision response compound-mesh"<<std::endl;
        #endif

        contact.m_vNormal = -contact.m_vNormal;

        RigidBody *subbody = contact.cbody0->rigidBodies_[contact.subId0];

        //radius of the component sphere
        Real R0 = subbody->shape_->getAABB().extents_[0];
        //radius of the second body (only if it is not a compound)
        //Real R1 = 0;

         //compound - boundarybox case; compound-plane is similar to this case
        Real xjxq = contact.m_dDistance;

        // VECTOR3 ztest = subbody->getTransformedPosition();

        Real myxi = std::max(R0 - xjxq, 0.0);

        //relative velocity of the contact point
        Real xidot = (subbody->velocity_- contact.m_pBody0->velocity_) * (-contact.m_vNormal);

        //compute normal force using linear viscoelastic model
        Real Fn = kN*myxi  + gammaN*xidot;

#ifdef DEBUG
        std::cout<<"kN*overlap: "<< kN*myxi << " dampening: " << gammaN*xidot <<std::endl;
        std::cout<<"overlap: "<< R0-xjxq << std::endl;
#endif

        //making sure Fn has a non-negative value;
        if (Fn < 1.0E-6 || myxi < 1.0E-12){
          Fn = 0.0;
        }

        VECTOR3 normalImpulse = (Fn * (contact.m_vNormal));

        //to compute tangential force, the relative velocity of the contact point  in regard to the whole bodies is needed
        //the relative positions of the contact point on each body
        VECTOR3 z = subbody->getTransformedPosition() + (R0 - myxi / 2.0) * (contact.m_vNormal);
        //z = contact.m_vPosition0;
        //VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->com_;

        MATRIX3X3 rot = contact.cbody0->getQuaternion().GetMatrix();
        MATRIX3X3 w2l = contact.cbody0->getQuaternion().GetMatrix();
        w2l.TransposeMatrix();

        VECTOR3 omega_world = rot * contact.cbody0->getAngVel();

        VECTOR3 relAngVelt = VECTOR3::Cross(-omega_world, z - contact.cbody0->com_);

        VECTOR3 relVelt = (-subbody->velocity_) + relAngVelt;

        VECTOR3 relAngVel = VECTOR3::Cross(-contact.cbody0->getAngVel(), z - contact.cbody0->com_);
        VECTOR3 relVel = (-subbody->velocity_) + relAngVel;

//        VECTOR3 tangentVel = relVel - (relVel * (contact.m_vNormal) * (contact.m_vNormal));
//        VECTOR3 tangentImpulse = tangentVel;
//
        VECTOR3 tangentVel_t = relVelt - (relVelt * (contact.m_vNormal) * (contact.m_vNormal));
        VECTOR3 tangentImpulse_t = tangentVel_t;

#ifdef DEBUG
        std::cout << "world omega: " << omega_world;
        std::cout << "local omega: " << contact.cbody0->getAngVel();
        std::cout << "world2local omega: " << w2l*omega_world;

        std::cout << "tangentVel: " << tangentVel;
        std::cout << "RelVel_t: " << relVelt*-contact.m_vNormal << std::endl;
#endif

//        Real Ft1 = mu * normalImpulse.mag();
//        Real Ft2 = gammaT * tangentVel.mag();
//
//        //tangential force is limited by coloumb`'s law of frictrion
//        Real min = -(std::min(Ft1, Ft2));
//
//        //normalize the vector
//        if (tangentVel.mag() != 0.0)
//        {
//          tangentImpulse = -1.0* tangentVel * (min / tangentVel.mag());
//        }
//
//        if (tangentVel_t.mag() != 0.0)
//        {
//          tangentImpulse_t = -1.0* tangentVel_t * (min / tangentVel_t.mag());
//        }

#ifdef DEBUG
        std::cout << "tangentVel: " << tangentVel;
        std::cout << "tangentImpulse: " << tangentImpulse;

        std::cout << "tangentVel_world: " << tangentVel_t;
        std::cout << "tangentImpulse_world: " << tangentImpulse_t;
#endif

        //compute the torques for the compound body
        VECTOR3 Torque0 = VECTOR3(0.0, 0.0, 0.0);
        VECTOR3 Torque0_t = VECTOR3(0.0, 0.0, 0.0);

        //and the force; they are only applied if there is an overlap, i.e. if myxi >0
        VECTOR3 Force0 = VECTOR3(0.0, 0.0, 0.0);

#ifdef DEBUG
        std::cout << "RelVel: " << relVel*-contact.m_vNormal << std::endl;
        std::cout << "RelVel_t: " << relVelt*-contact.m_vNormal << std::endl;
        std::cout << "RelVel_transl: " << (-subbody->velocity_)*-contact.m_vNormal << std::endl;
        std::cout << "RelVel_rot: " << (VECTOR3::Cross(-omega_world, z - contact.cbody0->com_))*-contact.m_vNormal << std::endl;

        std::cout << "contact point: " << z;
#endif
//        Torque0 = VECTOR3::Cross(z - contact.cbody0->com_, tangentImpulse);
//        //VECTOR3 z_t = rot * (z - contact.cbody0->com_);
//        Torque0_t = VECTOR3::Cross(z - contact.cbody0->com_, tangentImpulse_t);

        if (xjxq <= R0)
        {
          //Force0 = (normalImpulse + tangentImpulse) * contact.cbody0->invMass_;
          Force0 = (normalImpulse) * contact.cbody0->invMass_;
          //normal force may only be applied while relative normal velocity of the contact point
          // (relVel*n) is negative

          if (-xidot > 1.0E-6)// && (R0 - xjxq) < 0.025*R0)
          {
            Force0 = VECTOR3(0.0, 0.0, 0.0);
            Torque0 = VECTOR3(0.0, 0.0, 0.0);
            Torque0_t = VECTOR3(0.0, 0.0, 0.0);
          }



        }

#ifdef DEBUG
        //        std::cout<<"Overlap: "<< myxi <<std::endl;
        std::cout<<"normal Force: "<< Force0 <<std::endl;
        //        std::cout<<"velocity before: "<< contact.cbody0->velocity_ <<std::endl;
        std::cout<<"Torque0_mix "<< Torque0 <<std::endl;
        std::cout << "Torque0_world " << Torque0_t << std::endl;
        std::cout << "Torque0_trans " << w2l*Torque0 << std::endl;
#endif


        //for motionintegratorDEM based on taylor expansion, the applied forces for each component of a compound
        //are stored in the variables ComponentForces_ and ComponentTorques_ respectively.
        //these are then applied together with gravity within one timestep in the motionintegrator
        contact.cbody0->force_ += Force0;
        contact.cbody0->torque_ += Torque0_t;

        contact.cbody0->force_local_  += Force0;
        contact.cbody0->torque_local_ += Torque0_t;

        //now apply forces and torques
        //contact.cbody0->applyForces(Force0, Torque0, delta);
        //std::cout<<"velocity after: "<< contact.cbody0->velocity_ <<std::endl;

        //and to the boundary box
        //contact.m_pBody1->applyForces(Force1, Torque1);

    }
    //else its a rigid body-rigid body collision
    else
    {
      
    }
	}    
}

/**
this method uses the extended nonlinear model by Kruggel-Emden  */
void CollResponseDEM::ApplyImpulse1(CollisionInfo &ContactInfo, Real &delta)
{
	std::vector<Contact>::iterator iter;

	//model parameters for the normal force model

	Real kN_ex = 7.35E9;  //"spring stiffness" (scaled down to fit particle radius of 0.05 instead of 0.02)
	Real gammaN_ex = 5.76E8; //  == (3.59*10E2) ; //dampening constant (for a velocity-dependant damper)
	Real theta = 0.92781; //equals the normal force model by kuwabara and kono //0.92781 by experimental results

	//constants for tangential force, both range from 0.0 to 1.0
	Real mu = 0.0;//0.51;   //coefficient of static friction, 0.51 for brass
	Real gammaT = 0.0;// 1.0; //static friction strength

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


		//note that contact.m_pBody0 has the shapeId of the component, and not the whole compound
		if ((contact.cbody0->shapeId_ == RigidBody::COMPOUND) && contact.m_pBody1->shapeId_ == RigidBody::BOUNDARYBOX){

			RigidBody *subbody = contact.cbody0->rigidBodies_[contact.subId0];



			//radius of the component sphere 
			Real R0 = subbody->shape_->getAABB().extents_[0];
			//radius of the second body (only if it is not a compound)
			//Real R1 = 0;

			/**constants needed for computation of kN_ex
			R_eff = effective radius in case of compound-boundary collision is simply the radius R0 of the colliding component 
			nu_i = poisson ratio of body/component i of the collision
			E_eff = effective young's modulus given by 1/E_eff = (1-nu_i^2)/E_i + (1-nu_j^2)/E_j 
			E_i = young's modulus of body i
			
			with these, kN_ex can be computed via 
			kN_ex = 4/3 * E_eff * sqrt(R_eff)
			*/

			Real nu_i = 0.36;
            Real nu_j = 0.36;
			Real E_i = 9.6E10;
			Real E_j = E_i;
			Real young = (1 - nu_i) / E_i + (1 - nu_j) / E_j;
			
		    kN_ex = (4.0 / 3.0) * (1.0 / young )*std::sqrt(R0);
			
			
			//compound - boundarybox case; compound-plane is similar to this case 
			Real xjxq = contact.m_dDistance;

			// VECTOR3 ztest = subbody->getTransformedPosition();

			Real myxi = std::max(R0 - xjxq, 0.0);
		
			//relative velocity of the contact point
			Real xidot = -((subbody->velocity_) * (-contact.m_vNormal));

			
			//compute normal force using extnded nonlinear viscoelastic model
			Real Fn = kN_ex * std::pow(myxi, (3.0 / 2.0)) + gammaN_ex * xidot * std::pow(myxi, theta);
			
			//making sure Fn has a non-negative value;
			if (Fn < 1.0E-6 || myxi < 1.0E-12){
				Fn = 0.0;
			}
			VECTOR3 normalImpulse = -(Fn * (-contact.m_vNormal));

			//to compute tangential force, the relative velocity of the contact point  in regard to the whole bodies is needed 
			//the relative positions of the contact point on each body
			VECTOR3 z = subbody->getTransformedPosition() + (R0 - myxi / 2.0) * (-contact.m_vNormal);
			//VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->com_;

			VECTOR3 relAngVel = VECTOR3::Cross(-contact.cbody0->getAngVel(), z - contact.cbody0->com_);
			VECTOR3 relVel = (-subbody->velocity_) + relAngVel;

			VECTOR3 tangentVel = relVel - (relVel * (-contact.m_vNormal) * (-contact.m_vNormal));
			VECTOR3 tangentImpulse = tangentVel;

			Real Ft1 = mu * normalImpulse.mag();
			Real Ft2 = gammaT * tangentVel.mag();

			//tangential force is limited by coloumb`'s law of frictrion
			Real min = -(std::min(Ft1, Ft2));

			//normalize the vector
			if (tangentVel.mag() != 0.0){
				tangentImpulse = tangentVel * (min / tangentVel.mag());
			}

			//compute the torques for the compound body
			VECTOR3 Torque0 = VECTOR3(0.0, 0.0, 0.0);

			//and the force; they are only applied if there is an overlap, i.e. if myxi >0
			VECTOR3 Force0 = VECTOR3(0.0, 0.0, 0.0);

			if (myxi > 1.0E-6){
				Force0 = (normalImpulse + tangentImpulse) * contact.cbody0->invMass_;
				//normal force may only be applied while relative normal velocity of the contact point 
				// (relVel*n) is negative
				if (relVel*(-contact.m_vNormal) > 1.0E-6)
					Force0 = VECTOR3(0.0, 0.0, 0.0);

				Torque0 = VECTOR3::Cross(z - contact.cbody0->com_, tangentImpulse);
			}

			std::cout << "normal Force: " << Force0 << std::endl;
			std::cout << "velocity before: " << contact.cbody0->velocity_ << std::endl;

			//for motionintegratorDEM based on taylor expansion, the applied forces for each component of a compound 
			//are stored in the variables ComponentForces_ and ComponentTorques_ respectively. 
			//these are then applied together with gravity within one timestep in the motionintegrator
			//contact.cbody0->force_ += Force0;
			//contact.cbody0->torque_ += Torque0;


			//now apply forces and torques
			contact.cbody0->applyForces(Force0, Torque0, delta);
			std::cout<<"velocity after: "<< contact.cbody0->velocity_ <<std::endl;

			//and to the boundary box 
			//contact.m_pBody1->applyForces(Force1, Torque1);

		}

		//compound-compound collision 

		else if ((contact.cbody0->shapeId_ == RigidBody::COMPOUND) && (contact.m_pBody1->shapeId_ == RigidBody::COMPOUND)){

			RigidBody *subbody0 = contact.cbody0->rigidBodies_[contact.subId0];
			RigidBody *subbody1 = contact.cbody1->rigidBodies_[contact.subId1];

			//radius of the component sphere 
			Real R0 = subbody0->shape_->getAABB().extents_[0];
			Real R1 = subbody1->shape_->getAABB().extents_[0];

			//compute xi 
			Real xjxq = contact.m_dDistance;
			Real xi = std::max(R0 + R1 - xjxq, 0.0);
			//Real xi = R0 + R1 - xjxq;

			//contact.m_vNormal should have the correct sign, so we can obtain it directly from contact

			//compute xidot
			Real xidot = (subbody0->velocity_ - subbody1->velocity_) * (-contact.m_vNormal);

			//the contact point
			VECTOR3 z = subbody0->getTransformedPosition() + ((R0 - xi / 2.0) * (-contact.m_vNormal));

			//velocities of the contact points relative to the whole compounds
			VECTOR3 relAngVel = VECTOR3::Cross(contact.cbody1->getAngVel(), z - contact.cbody1->com_)
				- VECTOR3::Cross(contact.cbody0->getAngVel(), z - contact.cbody0->com_);

			VECTOR3 relVel = contact.cbody1->velocity_ - contact.cbody0->velocity_ + relAngVel;


			//now compute the normal and tangential force

			//normal force, linear viscoelastic model 
			Real Fn = kN_ex * std::pow(xi, 3.0 / 2.0) + gammaN_ex*xidot * std::pow(xi, theta);

			
			//making sure Fn has a non-negative value 
			if (Fn < 1.0E-6 || xi < 1.0E-12){
				Fn = 0.0;
			}
			VECTOR3 normalImpulse = -(Fn * (-contact.m_vNormal));

			//tangential force 
			VECTOR3 tangentVel = relVel - (relVel * (-contact.m_vNormal) * (-contact.m_vNormal));
			VECTOR3 tangentImpulse = tangentVel;

			Real Ft1 = mu * normalImpulse.mag();
			Real Ft2 = gammaT * tangentVel.mag();

			//tangential force is limited by coloumb`'s law of frictrion
			Real min = -(std::min(Ft1, Ft2));

			//normalize the vector
			if (tangentVel.mag() != 0.0){
				tangentImpulse = tangentVel * (min / tangentVel.mag());
			}

			//compute the torques for the compound body
			VECTOR3 Torque0 = VECTOR3(0.0, 0.0, 0.0);
			VECTOR3 Torque1 = VECTOR3(0.0, 0.0, 0.0);

			//and the force; they are only applied if there is an overlap, i.e. if myxi >0
			VECTOR3 Force0 = VECTOR3(0.0, 0.0, 0.0);
			VECTOR3 Force1 = VECTOR3(0.0, 0.0, 0.0);

			if (xi > 1.0E-6){
				Force0 = (normalImpulse + tangentImpulse) * contact.cbody0->invMass_;
				Force1 = (normalImpulse + tangentImpulse) * contact.cbody1->invMass_;

				//normal force may only be applied while relative normal velocity of the contact point 
				// (relVel*n) is negative

				if (relVel*(-contact.m_vNormal) > 1.0E-6)
					Force0 = VECTOR3(0.0, 0.0, 0.0);
				Force1 = VECTOR3(0.0, 0.0, 0.0);

				Torque0 = VECTOR3::Cross(z - contact.cbody0->com_, tangentImpulse);
				Torque1 = VECTOR3::Cross(z - contact.cbody1->com_, tangentImpulse);
			}


			//for motionintegratorDEM based on taylor expansion, the applied forces for each component of a compound 
			//are stored in the variables ComponentForces_ and ComponentTorques_ respectively. 
			//these are then applied together with gravity within one timestep in the motionintegrator
			contact.cbody0->force_ += Force0;
			contact.cbody0->torque_ += Torque0;
			contact.cbody1->force_ += Force1;
			contact.cbody1->torque_ += Torque1;


			//now apply forces and torques
			//contact.cbody0->applyForces(Force0, Torque0, delta);
			//contact.cbody1->applyForces(Force1, Torque1, delta);
			//std::cout<<"velocity after: "<< contact.cbody0->velocity_ <<std::endl;




		}


		//compound-sphere (or any other rigid body) case

		else if ((contact.cbody0->shapeId_ == RigidBody::COMPOUND) && (contact.m_pBody1->shapeId_ == RigidBody::SPHERE)){


			RigidBody *subbody0 = contact.cbody0->rigidBodies_[contact.subId0];
			RigidBody *body1 = contact.m_pBody1;

			//radius of the component sphere 
			Real R0 = subbody0->shape_->getAABB().extents_[0];
			Real R1 = body1->shape_->getAABB().extents_[0];

			//compute xi 
			Real xjxq = contact.m_dDistance;
			Real xi = std::max(R0 + R1 - xjxq, 0.0);
			//Real xi = R0 + R1 - xjxq;

			//contact.m_vNormal should have the correct sign, so we can obtain it directly from contact

			//compute xidot
			Real xidot = (subbody0->velocity_ - body1->velocity_) * (-contact.m_vNormal);

			//the contact point
			VECTOR3 z = subbody0->getTransformedPosition() + ((R0 - xi / 2.0) * (-contact.m_vNormal));

			//velocities of the contact points relative to the whole compounds
			VECTOR3 relAngVel = VECTOR3::Cross(contact.m_pBody0->getAngVel(), z - contact.m_pBody0->com_)
				- VECTOR3::Cross(contact.cbody0->getAngVel(), z - contact.cbody0->com_);

			VECTOR3 relVel = contact.m_pBody1->velocity_ - contact.cbody0->velocity_ + relAngVel;


			//now compute the normal and tangential force

			//normal force, linear viscoelastic model 
			Real Fn = kN_ex * std::pow(xi, 3.0 / 2.0) + gammaN_ex*xidot * std::pow(xi, theta);

			//making sure Fn has a non-negative value 
			if (Fn < 1.0E-6 || xi < 1.0E-12){
				Fn = 0.0;
			}
			VECTOR3 normalImpulse = -(Fn * (-contact.m_vNormal));

			//tangential force 
			VECTOR3 tangentVel = relVel - (relVel * (-contact.m_vNormal) * (-contact.m_vNormal));
			VECTOR3 tangentImpulse = tangentVel;

			Real Ft1 = mu * normalImpulse.mag();
			Real Ft2 = gammaT * tangentVel.mag();

			//tangential force is limited by coloumb`'s law of frictrion
			Real min = -(std::min(Ft1, Ft2));

			//normalize the vector
			if (tangentVel.mag() != 0.0){
				tangentImpulse = tangentVel * (min / tangentVel.mag());
			}

			//compute the torques for the compound body
			VECTOR3 Torque0 = VECTOR3(0.0, 0.0, 0.0);
			VECTOR3 Torque1 = VECTOR3(0.0, 0.0, 0.0);

			//and the force; they are only applied if there is an overlap, i.e. if myxi >0
			VECTOR3 Force0 = VECTOR3(0.0, 0.0, 0.0);
			VECTOR3 Force1 = VECTOR3(0.0, 0.0, 0.0);

			if (xi > 1.0E-6){
				Force0 = (normalImpulse + tangentImpulse) * contact.cbody0->invMass_;
				Force1 = (normalImpulse + tangentImpulse) * contact.m_pBody0->invMass_;

				//normal force may only be applied while relative normal velocity of the contact point 
				// (relVel*n) is negative

				if (relVel*(-contact.m_vNormal) > 1.0E-6)
					Force0 = VECTOR3(0.0, 0.0, 0.0);
				Force1 = VECTOR3(0.0, 0.0, 0.0);

				Torque0 = VECTOR3::Cross(z - contact.cbody0->com_, tangentImpulse);
				Torque1 = VECTOR3::Cross(z - contact.m_pBody1->com_, tangentImpulse);
			}


			//for motionintegratorDEM based on taylor expansion, the applied forces for each component of a compound 
			//are stored in the variables ComponentForces_ and ComponentTorques_ respectively. 
			//these are then applied together with gravity within one timestep in the motionintegrator
			contact.cbody0->force_ += Force0;
			contact.cbody0->torque_ += Torque0;
			contact.m_pBody1->force_ += Force1;
			contact.m_pBody1->torque_ += Torque1;


			//now apply forces and torques
			//contact.cbody0->applyForces(Force0, Torque0, delta);
			//contact.m_pBody1->applyForces(Force1, Torque1, delta);
			//std::cout<<"velocity after: "<< contact.cbody0->velocity_ <<std::endl;


		}

		//else its a rigid body-rigid body collision
		else{

		}




	}

}



}
