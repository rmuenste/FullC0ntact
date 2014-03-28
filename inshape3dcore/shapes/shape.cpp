#include "shape.h"

namespace i3d {

template<class T>
Shape<T>::Shape(void)
{
}

template<class T>
Shape<T>::~Shape(void)
{
}

template<class T>
AABB3<T> Shape<T>::getAABB()
{
	return AABB3<T>();
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class Shape<float>;

template class Shape<double>;
//----------------------------------------------------------------------------

}