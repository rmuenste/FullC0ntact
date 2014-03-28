/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "ugsizeheuristicstd.h"

namespace i3d {

UniformGridSizeHeuristicst1::UniformGridSizeHeuristicst1()
{
  nLevels_ = 0;
}

UniformGridSizeHeuristicst1::UniformGridSizeHeuristicst1(const UniformGridSizeHeuristic& other)
{

}

UniformGridSizeHeuristicst1::~UniformGridSizeHeuristicst1()
{

}

void UniformGridSizeHeuristicst1::computeCellSizes(std::list< std::pair< Real, int > >& sizes, Real sizeHint)
{
  // The heuristic searches a cell of  a size that is sizeHint * sizeOfFirstCell and makes
  // this the size of the first grid level. From there it proceeds to build the grid
  // in an analogous fashion
  double factor = sizeHint;
  std::list< std::pair<Real,int> >::iterator liter = sizes.begin();

  double tsize = factor * ((*liter).first);
  liter++;
  int levels=0;
  int elemPerLevel=1;
  double lastsize=0.0;
  double dsize=0.0;
  for(;liter!=sizes.end();liter++)
  {
    dsize=((*liter).first);
    if(dsize > tsize)
    {
      gridSizes_.push_back(lastsize);
      tsize=factor*dsize;
      lastsize=dsize;
      distribution_.push_back(elemPerLevel);
      elemPerLevel=1;
    }
    else
    {
      lastsize=dsize;
      elemPerLevel++;
    }
  }

  gridSizes_.push_back(lastsize);
  distribution_.push_back(elemPerLevel);
  nLevels_=distribution_.size();

}

}