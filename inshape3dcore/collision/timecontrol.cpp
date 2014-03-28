#include "timecontrol.h"

namespace i3d {

TimeControl::TimeControl(void)
{
	m_dDeltaT    = Real(0);
	m_dTime      = Real(0);
	m_iTimeStep  = 0;
	m_dCautious  = 0.0;
	m_dPreferred = 0.0;
	m_dReduced   = 0.0;
}

TimeControl::~TimeControl(void)
{
}

TimeControl::TimeControl(Real dDeltaT)
{
	m_dDeltaT = dDeltaT;
	m_dTime   = Real(0);
	m_iTimeStep  = 0;
	m_dCautious  = 0.0;
	m_dPreferred = 0.0;
	m_dReduced   = 0.0;
}

}
