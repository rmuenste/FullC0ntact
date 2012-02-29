#include "sphere.h"

namespace i3d {

template<class T>
CSphere<T>::CSphere(void)
{
}

template<class T>
CSphere<T>::~CSphere(void)
{
}

template<class T>
CAABB3<T> CSphere<T>::GetAABB(void)
{
	return CAABB3<T>(m_vCenter,m_Rad);
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CSphere<float>;

template class CSphere<double>;
//----------------------------------------------------------------------------

}