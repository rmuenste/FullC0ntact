#include "sphere.h"

namespace i3d {

template<class T>
Sphere<T>::Sphere(void)
{
}

template<class T>
Sphere<T>::~Sphere(void)
{
}

template<class T>
CAABB3<T> Sphere<T>::getAABB(void)
{
	return CAABB3<T>(center_,radius_);
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class Sphere<float>;

template class Sphere<double>;
//----------------------------------------------------------------------------

}