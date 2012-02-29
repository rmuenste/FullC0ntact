#include "ellipsoid.h"

namespace i3d {

template<class T>
CEllipsoid<T>::CEllipsoid(void)
{
}
template<class T>
CEllipsoid<T>::~CEllipsoid(void)
{
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CEllipsoid<float>;

template class CEllipsoid<double>;
//----------------------------------------------------------------------------

}