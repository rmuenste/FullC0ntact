#include "boundarycyl.h"

namespace i3d {

template <class T>
BoundaryCyl<T>::BoundaryCyl(void)
{

}

template <class T>
BoundaryCyl<T>::BoundaryCyl(const CVector3<T> &vOrigin, const T extends[3])
{

}

template <class T>
BoundaryCyl<T>::~BoundaryCyl(void)
{

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class BoundaryCyl<float>;

template class BoundaryCyl<double>;
//----------------------------------------------------------------------------

}