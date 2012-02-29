/***************************************************************************
 *   Copyright (C) 2006-2009 by Raphael Münster   *
 *   raphael@Cortez   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef _TIMER_H_
#define _TIMER_H_

//====================================================
//					INCLUDES
//====================================================

#include <windows.h>
#include <mmsystem.h>

//====================================================
//					DEFINES
//====================================================



//====================================================
//					GLOBALS
//====================================================


class CTimer
{
public:
	CTimer(void);
	~CTimer(void);

	inline void Start()
	{
		QueryPerformanceCounter((LARGE_INTEGER*) &lastTime);
	}

	inline double Elapsed()
	{
		QueryPerformanceCounter((LARGE_INTEGER*) &currentTime);
		return m_dTimeElapsed = (currentTime - lastTime) * m_dTimeScale;
	}

	inline void Stop()
	{
		lastTime=(LONGLONG)0.0;
	}


	LONGLONG frequency, lastTime, currentTime;
	double m_dTimeElapsed, m_dTimeScale;

};

#endif