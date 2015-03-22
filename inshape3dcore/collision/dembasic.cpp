#include "dembasic.h"
#include <iostream>
#include <sphere.h>

namespace i3d {

  void DemBasic::evalCompoundBox(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact)
  {

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
    Real xidot = (subbody->velocity_ - contact.m_pBody0->velocity_) * (-contact.m_vNormal);

    //compute normal force using linear viscoelastic model
    Real Fn = kN*myxi + gammaN*xidot;

#ifdef DEBUG
    std::cout << "kN*overlap: " << kN*myxi << " dampening: " << gammaN*xidot << std::endl;
    std::cout << "overlap: " << R0 - xjxq << std::endl;
#endif

    //making sure Fn has a non-negative value;
    if (Fn < 1.0E-6 || myxi < 1.0E-12){
      Fn = 0.0;
    }

    VECTOR3 normalImpulse = Fn * contact.m_vNormal;

    //to compute tangential force, the relative velocity of the contact point  in regard to the whole bodies is needed
    //the relative positions of the contact point on each body
    VECTOR3 z = subbody->getTransformedPosition() + (R0 - myxi / 2.0) * (-contact.m_vNormal);

    MATRIX3X3 rot = contact.cbody0->getQuaternion().GetMatrix();
    MATRIX3X3 w2l = contact.cbody0->getQuaternion().GetMatrix();
    w2l.TransposeMatrix();

    //transform the local angular velocity to world space
    VECTOR3 omega_world = rot * contact.cbody0->getAngVel();

    VECTOR3 relAngVelt = VECTOR3::Cross(-omega_world, z - contact.cbody0->com_);

    VECTOR3 relVelt = (-subbody->velocity_) + relAngVelt;

    VECTOR3 tangentVel_t = relVelt - (relVelt * (contact.m_vNormal) * (contact.m_vNormal));
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

    Torque0_t = VECTOR3::Cross(z - contact.cbody0->com_, tangentImpulse_t);

    if (xjxq <= R0)
    {
      Force0 = (normalImpulse + tangentImpulse_t) * contact.cbody0->invMass_;
      //Force0 = (normalImpulse) * contact.cbody0->invMass_;
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
    std::cout << "normal Force: " << Force0 << std::endl;
    //        std::cout<<"velocity before: "<< contact.cbody0->velocity_ <<std::endl;
    std::cout << "Torque0_mix " << Torque0 << std::endl;
    std::cout << "Torque0_world " << Torque0_t << std::endl;
    std::cout << "Torque0_trans " << w2l*Torque0 << std::endl;
#endif


    //for motionintegratorDEM based on taylor expansion, the applied forces for each component of a compound
    //are stored in the variables ComponentForces_ and ComponentTorques_ respectively.
    //these are then applied together with gravity within one timestep in the motionintegrator
    contact.cbody0->force_ += Force0;
    contact.cbody0->torque_ += Torque0_t;

    contact.cbody0->force_local_ += Force0;
    contact.cbody0->torque_local_ += Torque0_t;

  }

  void DemBasic::evalCompoundMesh(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact)
  {

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
    Real xidot = subbody->velocity_ * contact.m_vNormal;

    //compute normal force using linear viscoelastic model
    Real Fn = kN*myxi + gammaN*xidot;

#ifdef DEBUG						
    std::cout << "kN*overlap: " << kN*myxi << " dampening: " << gammaN*xidot << std::endl;
    std::cout << "overlap: " << R0 - xjxq << std::endl;
#endif

    //making sure Fn has a non-negative value;
    if (Fn < 1.0E-6 || myxi < 1.0E-12){
      Fn = 0.0;
    }

    VECTOR3 normalImpulse = Fn * contact.m_vNormal;

    //This line is kinda suspicious!!!
    VECTOR3 z = subbody->getTransformedPosition() + (R0 - myxi / 2.0) * (-contact.m_vNormal);

    MATRIX3X3 rot = contact.cbody0->getQuaternion().GetMatrix();
    MATRIX3X3 w2l = contact.cbody0->getQuaternion().GetMatrix();
    w2l.TransposeMatrix();

    //transform omega to world coordinates
    VECTOR3 omega_w = rot * contact.cbody0->getAngVel();

    VECTOR3 relAngVel_w = VECTOR3::Cross(-omega_w, z - contact.cbody0->com_);

    VECTOR3 relVel_w = (-subbody->velocity_) + relAngVel_w;

    VECTOR3 tangentVel_w = relVel_w - (relVel_w * (contact.m_vNormal) * (contact.m_vNormal));
    VECTOR3 tangentImpulse_w = tangentVel_w;

#ifdef DEBUG						
    std::cout << "world omega: " << omega_world;
    std::cout << "local omega: " << contact.cbody0->getAngVel();
    std::cout << "world2local omega: " << w2l*omega_world;

    std::cout << "tangentVel: " << tangentVel;
    std::cout << "RelVel_t: " << relVelt*-contact.m_vNormal << std::endl;
#endif

    Real Ft1 = mu * normalImpulse.mag();
    Real Ft2 = gammaT * tangentVel_w.mag();

    //tangential force is limited by coloumb`'s law of frictrion
    Real min = -(std::min(Ft1, Ft2));

    if (tangentVel_w.mag() != 0.0)
    {
      tangentImpulse_w = -1.0* tangentVel_w * (min / tangentVel_w.mag());
    }

#ifdef DEBUG						
    std::cout << "tangentVel: " << tangentVel;
    std::cout << "tangentImpulse: " << tangentImpulse;

    std::cout << "tangentVel_world: " << tangentVel_w;
    std::cout << "tangentImpulse_world: " << tangentImpulse_w;
#endif

    //compute the torques for the compound body
    VECTOR3 Torque0 = VECTOR3(0.0, 0.0, 0.0);
    VECTOR3 Torque0_w = VECTOR3(0.0, 0.0, 0.0);

    //and the force; they are only applied if there is an overlap, i.e. if myxi >0
    VECTOR3 Force0 = VECTOR3(0.0, 0.0, 0.0);

#ifdef DEBUG						
    std::cout << "RelVel: " << relVel*-contact.m_vNormal << std::endl;
    std::cout << "RelVel_t: " << relVelt*-contact.m_vNormal << std::endl;
    std::cout << "RelVel_transl: " << (-subbody->velocity_)*-contact.m_vNormal << std::endl;
    std::cout << "RelVel_rot: " << (VECTOR3::Cross(-omega_world, z - contact.cbody0->com_))*-contact.m_vNormal << std::endl;

    std::cout << "contact point: " << z;
#endif

    Torque0_w = VECTOR3::Cross(z - contact.cbody0->com_, tangentImpulse_w);

    if (xjxq <= R0)
    {
      Force0 = (normalImpulse + tangentImpulse_w) * contact.cbody0->invMass_;
      //normal force may only be applied while relative normal velocity of the contact point
      // (relVel*n) is negative

      if (relVel_w*(-contact.m_vNormal) > 1.0E-6)// && (R0 - xjxq) < 0.025*R0)
      {
        Force0 = VECTOR3(0.0, 0.0, 0.0);
        Torque0 = VECTOR3(0.0, 0.0, 0.0);
        Torque0_w = VECTOR3(0.0, 0.0, 0.0);
      }

    }

#ifdef DEBUG						
    //        std::cout<<"Overlap: "<< myxi <<std::endl;
    std::cout << "normal Force: " << Force0 << std::endl;
    //        std::cout<<"velocity before: "<< contact.cbody0->velocity_ <<std::endl;
    std::cout << "Torque0_mix " << Torque0 << std::endl;
    std::cout << "Torque0_world " << Torque0_t << std::endl;
    std::cout << "Torque0_trans " << w2l*Torque0 << std::endl;
#endif


    //for motionintegratorDEM based on taylor expansion, the applied forces for each component of a compound
    //are stored in the variables ComponentForces_ and ComponentTorques_ respectively.
    //these are then applied together with gravity within one timestep in the motionintegrator
    contact.cbody0->force_ += Force0;
    contact.cbody0->torque_ += Torque0_w;

    contact.cbody0->force_local_ += Force0;
    contact.cbody0->torque_local_ += Torque0_w;

  }

  void DemBasic::evalCompoundCompound(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact)
  {

#ifdef DEBUG						
    std::cout << "Collision response compound-compound" << std::endl;
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

    //compute xidot
    Real xidot = (subbody0->velocity_ - subbody1->velocity_) * (-contact.m_vNormal);

    //the contact point
    VECTOR3 ztest = subbody0->getTransformedPosition();
    VECTOR3 ztest2 = ((R0 - xi / 2.0) * (-contact.m_vNormal));
    VECTOR3 z = subbody0->getTransformedPosition() + ((R0 - xi / 2.0) * (-contact.m_vNormal));

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
    std::cout << "Particle-Particle: kN*overlap: " << kN*xi << " dampening: " << gammaN*xidot << std::endl;
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
    std::cout << "simulation_time: " << m_pWorld->timeControl_->GetTime() << " overlap: " << xi << std::endl;
#endif

    //for motionintegratorDEM based on taylor expansion, the applied forces for each component of a compound
    //are added onto the variables force_ and torque_ since the force and torque acting on a compound 
    //is the sum of the forces and torques of each component
    //these are then applied together with gravity within one timestep in the motionintegrator
    contact.cbody0->force_ += Force0;
    contact.cbody0->torque_local_ += Torque0;
    contact.cbody1->force_ += Force1;
    contact.cbody1->torque_local_ += Torque1;

  }

  void DemBasic::evalCompoundBoundary(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact)
  {


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
    Real Fn = kN*myxi + gammaN*xidot;

    //making sure Fn has a non-negative value;
    if (Fn < 1.0E-6 || myxi < 1.0E-12)
    {
      Fn = 0.0;
    }

#ifdef DEBUG
    std::cout << "simtime: " << m_pWorld->timeControl_->GetTime() << " overlap: " << myxi << std::endl;
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
    if (tangentVel.mag() != 0.0)
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

  }

}