#ifndef COMPARE_H
#define COMPARE_H

#include <collisioninfo.h>
#include <unstructuredgrid.h>

namespace i3d {

class CmpDist
{
public:
	bool operator() (const DistQueueEntry &first, const DistQueueEntry &second)
	{
		// return true if first has lower priority than second
		// false otherwise
		if(first.pTraits->distance < second.pTraits->distance)
			return true;
		else
			return false;
	}
};

///@cond
}

#endif
