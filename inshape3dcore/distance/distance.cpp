#include "distance.h"

namespace i3d {

template<typename T>
CDistance<T>::CDistance(void)
{
}

template<typename T>
CDistance<T>::~CDistance(void)
{
}

template<typename T>
T CDistance<T>::ComputeDistanceSqr()
{
	return T(0);
}

template<typename T>
T CDistance<T>::ComputeDistance()
{
	return T(0);
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistance<float>;

template class CDistance<double>;
//----------------------------------------------------------------------------

}