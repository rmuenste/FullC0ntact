#include "response.h"

namespace i3d {

CResponse::CResponse(void)
{
}

CResponse::~CResponse(void)
{
}

CResponse::CResponse(const VECTOR3 &v1, const VECTOR3 &v2)
{
	this->m_vFW1 = v1;
	this->m_vFW2 = v2;
	this->iID1 = 0;
	this->iID2 = 0;
}

CResponse::CResponse(const VECTOR3 &v1,const VECTOR3 &v2,int i1, int i2)
{
	this->m_vFW1 = v1;
	this->m_vFW2 = v2;
	this->iID1 = i1;
	this->iID2 = i2;
}

CResponse::CResponse(const CResponse &copy)
{
	this->m_vFW1 = copy.m_vFW1;
	this->m_vFW2 = copy.m_vFW2;
	this->m_vOmega1 = copy.m_vOmega1;
	this->m_vOmega2 = copy.m_vOmega2;
	this->iID1 = copy.iID1;
	this->iID2 = copy.iID2;
}

CResponse::CResponse(const VECTOR3 &v1,const VECTOR3 &v2, const VECTOR3 &omega1,const VECTOR3 &omega2, int i1, int i2)
{
	this->m_vFW1 = v1;
	this->m_vFW2 = v2;
	this->m_vOmega1 = omega1;
	this->m_vOmega2 = omega2;
	this->iID1 = i1;
	this->iID2 = i2;
}

}