#include "ellipsoid.h"

namespace i3d {

template<class T>
Ellipsoid<T>::Ellipsoid(void)
{
}
template<class T>
Ellipsoid<T>::~Ellipsoid(void)
{
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class Ellipsoid<float>;

template class Ellipsoid<double>;
//----------------------------------------------------------------------------

}