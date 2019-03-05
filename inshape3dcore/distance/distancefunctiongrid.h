/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#ifndef DISTANCEFUNCTIONGRID_H
#define DISTANCEFUNCTIONGRID_H

//===================================================
//					DEFINITIONS
//===================================================


//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include <vector>
#include <limits>
#include <vector3.h>
#include <unstructuredgrid.h>
#include <geom_config.hpp>
#include <distancemap.h>
#include <memory>

namespace i3d {

/**
* @brief Base class for distance function on a grid
*
* Computes a distance function on a grid
*/  
template<typename T>
class DistanceFuncGrid
{
public:
	DistanceFuncGrid(void);
	DistanceFuncGrid(UnstructuredGrid<T,DTraits> *pGrid);

	virtual ~DistanceFuncGrid(void);

	virtual void ComputeDistance();

	UnstructuredGrid<T,DTraits> *m_pGrid;

};

}

#endif
