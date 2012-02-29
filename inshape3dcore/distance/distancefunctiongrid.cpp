//===================================================
//					INCLUDES
//===================================================
#include "distancefunctiongrid.h"

namespace i3d {

template<typename T>
CDistanceFuncGrid<T>::CDistanceFuncGrid()
{

}

template<typename T>
CDistanceFuncGrid<T>::CDistanceFuncGrid(CUnstructuredGrid<T,DTraits> *pGrid) : m_pGrid(pGrid)
{
	//m_pGrid = pGrid;
}

template<typename T>
CDistanceFuncGrid<T>::~CDistanceFuncGrid()
{

}

template<typename T>
void CDistanceFuncGrid<T>::ComputeDistance()
{

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceFuncGrid<float>;

template class CDistanceFuncGrid<double>;
//----------------------------------------------------------------------------

}