#include "shape.h"

namespace i3d {

template<class T>
CShape<T>::CShape(void)
{
}

template<class T>
CShape<T>::~CShape(void)
{
}

template<class T>
CAABB3<T> CShape<T>::GetAABB()
{
	return CAABB3<T>();
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CShape<float>;

template class CShape<double>;
//----------------------------------------------------------------------------

}