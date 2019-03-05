//===================================================
//					INCLUDES
//===================================================
#include "distancefunctiongrid.h"

namespace i3d {

template<typename T>
DistanceFuncGrid<T>::DistanceFuncGrid()
{

}

template<typename T>
DistanceFuncGrid<T>::DistanceFuncGrid(UnstructuredGrid<T,DTraits> *pGrid) : m_pGrid(pGrid)
{
	//m_pGrid = pGrid;
}

template<typename T>
DistanceFuncGrid<T>::~DistanceFuncGrid()
{

}

template<typename T>
void DistanceFuncGrid<T>::ComputeDistance()
{

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class DistanceFuncGrid<float>;

template class DistanceFuncGrid<double>;
//----------------------------------------------------------------------------

}