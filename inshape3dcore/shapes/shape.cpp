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
CAABB3<T> Shape<T>::GetAABB()
{
	return CAABB3<T>();
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class Shape<float>;

template class Shape<double>;
//----------------------------------------------------------------------------

}