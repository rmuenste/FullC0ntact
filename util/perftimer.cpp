#include "perftimer.h"
#include <iostream>

CPerfTimer::CPerfTimer(void)
{
}

CPerfTimer::~CPerfTimer(void)
{
}

void CPerfTimer::Start(void)
{
#ifdef WIN32
  LARGE_INTEGER li;
  if(!QueryPerformanceFrequency(&li))
    std::cout<<"QueryPerformanceFrequency failed"<<std::endl;

  m_dFrq = double(li.QuadPart)/1000.0;
  QueryPerformanceCounter(&li);
  m_iStart = li.QuadPart;
#elif defined (__APPLE__)
  return;
#else
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &m_tstart);

#endif
}

// Returns the e
double CPerfTimer::GetTime(void)
{
#ifdef WIN32
  LARGE_INTEGER li;
  QueryPerformanceCounter(&li);
  return (li.QuadPart-m_iStart)/m_dFrq;
#elif defined (__APPLE__)
  return 0.0;
#else
clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &m_tend);
timespec temp;
if ((m_tend.tv_nsec-m_tstart.tv_nsec)<0) {
	temp.tv_sec = m_tend.tv_sec-m_tstart.tv_sec-1;
	temp.tv_nsec = 1000000000+m_tend.tv_nsec-m_tstart.tv_nsec;
} else {
	temp.tv_sec = m_tend.tv_sec-m_tstart.tv_sec;
	temp.tv_nsec = m_tend.tv_nsec-m_tstart.tv_nsec;
}
// Returns the elapsed time in miliseconds seconds
return temp.tv_sec+temp.tv_nsec/1000000.0;
#endif

}

int CPerfTimer::Stop(void)
{
	return 0;
}
