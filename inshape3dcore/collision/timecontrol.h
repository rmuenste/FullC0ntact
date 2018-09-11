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

#ifndef TIMECONTROL_H
#define TIMECONTROL_H
#include <mymath.h>

namespace i3d {

class TimeControl
{
public:
	TimeControl(void);
	TimeControl(Real dDeltaT);
	~TimeControl(void);

	inline Real GetDeltaT() {return m_dDeltaT;};
	inline Real GetTime() {return m_dTime;};

	inline void SetDeltaT(Real dDeltaT) {m_dDeltaT=dDeltaT;};
	inline void SetPreferred() {m_dDeltaT=m_dPreferred;};
	inline void SetCautious() {m_dDeltaT=m_dCautious;};
	inline void SetReduced() {m_dDeltaT=m_dReduced;};
	inline void SetTime(Real dTime) {m_dTime=dTime;};
	inline void SetTimeStep(int iTimeStep) {m_iTimeStep = iTimeStep;};
	inline void step(Real dt) {m_dTime += dt;};

	inline void SetPreferredTimeStep(Real dPreferredTimestep) {m_dPreferred=dPreferredTimestep;};
	inline void SetCautiousTimeStep(Real dCautiousTimestep) {m_dCautious=dCautiousTimestep;};
	inline void SetReducedTimeStep(Real dReducedTimestep) {m_dReduced=dReducedTimestep;};

	inline Real GetPreferredTimeStep() {return m_dPreferred;};
	inline Real GetCautiousTimeStep() {return m_dCautious;};
	inline Real GetReducedTimeStep() {return m_dReduced;};
	inline int  GetTimeStep() {return m_iTimeStep;};	
	inline Real getTime() {return m_dTime;};


	Real m_dDeltaT;
	Real m_dTime;
	Real m_dPreferred;
	Real m_dCautious;
	Real m_dReduced;
	int  m_iTimeStep;

};

}

#endif
