#ifndef _PERFTIMER_H_
#define _PERFTIMER_H_

//====================================================
//					INCLUDES
//====================================================


//====================================================
//					DEFINES
//====================================================



//====================================================
//					GLOBALS
//====================================================

/** \brief A brief description of MyClass.
 *
 * A more extensive description of MyClass.
 */
//====================================================
//					CLASS DEFINITION
//====================================================
#ifdef WIN32
#define NOMINMAX
#include <Windows.h>
#else
#include <time.h>
#include <sys/types.h>
#endif
class CPerfTimer
{
public:


	/** @brief A brief description of myProcedure().
	 *
	 * A more extensive description of myProcedure().
	 */
	CPerfTimer(void);

	/** @brief A brief description of myProcedure().
	 *
	 * A more extensive description of myProcedure().
	 */
	~CPerfTimer(void);


	void Start(void);

  double GetTime();

	int Stop(void);

  double m_dFrq;

#ifndef WIN32
timespec m_tstart;
timespec m_tend;
#else
  __int64 m_iStart;
#endif

};

#endif
