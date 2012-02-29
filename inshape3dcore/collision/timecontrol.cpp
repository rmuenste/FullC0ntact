#include "timecontrol.h"

namespace i3d {

CTimeControl::CTimeControl(void)
{
	m_dDeltaT    = Real(0);
	m_dTime      = Real(0);
	m_iTimeStep  = 0;
	m_dCautious  = 0.0;
	m_dPreferred = 0.0;
	m_dReduced   = 0.0;
}

CTimeControl::~CTimeControl(void)
{
}

CTimeControl::CTimeControl(Real dDeltaT)
{
	m_dDeltaT = dDeltaT;
	m_dTime   = Real(0);
	m_iTimeStep  = 0;
	m_dCautious  = 0.0;
	m_dPreferred = 0.0;
	m_dReduced   = 0.0;
}

}
