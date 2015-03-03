#include "contact.h"

namespace i3d {

Contact::Contact(void)
{

	m_dDistance                  = 0.0;
	m_vNormal                    = VECTOR3(0,0,0);
	vn                           = 0.0;
  m_dAccumulatedNormalImpulse  = 0.0;
  m_dAccumulatedTangentImpulseU= 0.0;
  m_dAccumulatedTangentImpulseV= 0.0;
  m_dBiasImpulse               = 0.0;
  m_dMassNormal                = 0.0;
  m_dMassTangentU              = 0.0;
  m_dMassTangentV              = 0.0;
  m_dRestitution               = 0.0;
	m_vPosition0                 = VECTOR3(0,0,0);
	m_vPosition1                 = VECTOR3(0,0,0);
	m_pBody0                     = NULL;
	m_pBody1                     = NULL;
	id0                          = -1;
	id1                          = -1;
  m_dPenetrationDepth          = 0.0;
  contactId_                   = -1;
  type0                   = -1;
  type1                   = -1;

}

Contact::~Contact(void)
{
}

Contact::Contact(const Contact &copy)
{

	m_dDistance                  = copy.m_dDistance;
	m_vNormal                    = copy.m_vNormal;
	vn                           = copy.vn;
  m_dAccumulatedNormalImpulse  = copy.m_dAccumulatedNormalImpulse;
  m_dAccumulatedTangentImpulseU= copy.m_dAccumulatedTangentImpulseU;
  m_dAccumulatedTangentImpulseV= copy.m_dAccumulatedTangentImpulseV;
  m_dBiasImpulse               = copy.m_dBiasImpulse;
  m_dMassNormal                = copy.m_dMassNormal;
  m_dMassTangentU              = copy.m_dMassTangentU;
  m_dMassTangentV              = copy.m_dMassTangentV;
  m_dRestitution               = copy.m_dRestitution;
	m_vPosition0                 = copy.m_vPosition0;
	m_vPosition1                 = copy.m_vPosition1;
	m_pBody0                     = copy.m_pBody0;
	m_pBody1                     = copy.m_pBody1;
	id0                          = copy.id0;
	id1                          = copy.id1;
  m_dPenetrationDepth          = copy.m_dPenetrationDepth;
  subId0                       = copy.subId0;
  subId1                       = copy.subId1;

  type0                       = copy.type0;
  type1                       = copy.type1;

  cbody0 = copy.cbody0;
  cbody1 = copy.cbody1;
  m_iState                     = copy.m_iState;
  m_iPrevState                 = copy.m_iPrevState;
  m_iTimeStamp                 = copy.m_iTimeStamp;
  m_iPrevTimeStamp             = copy.m_iPrevTimeStamp;  

}

Real Contact::GetSign(RigidBody *pBody)
{
  if(pBody == m_pBody0)
  {
    return Real(1.0);
  }
  else if(pBody == m_pBody1)
  {
    return Real(-1.0);
  }
  else
  {
    return Real(0.0);
  }
}

}
