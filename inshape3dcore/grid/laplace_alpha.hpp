
/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

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
#ifndef LAPLACE_ALPHA_HPP
#define LAPLACE_ALPHA_HPP

//===================================================
//                     INCLUDES
//===================================================
#include <list>
#include <aabb3.h>
#include <unstructuredgrid.h> 
#include <meshobject.h>
#include <distancegridcgal.hpp>

namespace i3d {

/**
 * @brief Base class for non-pde based smoothers
 * 
 */
template<class T>  
class LaplaceAlpha
{
public:
  LaplaceAlpha(){};
  
  ~LaplaceAlpha(){};

  LaplaceAlpha(UnstructuredGrid<T, DTraits> *grid, MeshObject<T, cgalKernel> *geom, int iterations = 10);

  int getIterations(){ return iterations_; };

  void setIterations(int iterations){ iterations_ = iterations; };

  UnstructuredGrid<T, DTraits>* getGrid(){ return grid_; };

  void setGrid(UnstructuredGrid<T, DTraits>* grid) { grid_ = grid; };

  void smooth();

  void smoothMultiLevel();

  UnstructuredGrid<T, DTraits> *grid_;

  MeshObject<T, cgalKernel> *geom_;

  int iterations_;

private:

  void smoothingKernel(Vector3<T> *coords, int level);

  void prolongate(int level);

  void calculateVolumeWeight();

  void calculateVertexWeight();

  T weightCalculation(T d);

  std::vector<T> vertexWeight_;
  std::vector<T> cumulativeWeight_;
  std::vector<T> volWeight_;
  std::vector<int> n;
  

};

}
#endif
