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
AABB3<T> Sphere<T>::getAABB(void)
{
	return AABB3<T>(center_,radius_);
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class Sphere<float>;

template class Sphere<double>;
//----------------------------------------------------------------------------

}