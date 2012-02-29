#include "Timer.h"

CTimer::CTimer(void)
{
	lastTime=0.0;
	currentTime=0.0;
	QueryPerformanceFrequency((LARGE_INTEGER*) &frequency);
	m_dTimeScale = 1.0/frequency;
}

CTimer::~CTimer(void)
{
}



