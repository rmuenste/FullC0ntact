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


#include "ugsizeheuristicgrd.h"

namespace i3d {

UniformGridSizeHeuristic2::UniformGridSizeHeuristic2()
{
  nLevels_ = 0;
}

UniformGridSizeHeuristic2::UniformGridSizeHeuristic2(const UniformGridSizeHeuristic& other)
{

}

UniformGridSizeHeuristic2::~UniformGridSizeHeuristic2()
{

}

void UniformGridSizeHeuristic2::computeCellSizes(std::list< std::pair< Real, int > >& sizes, Real sizeHint)
{
  
  double factor = sizeHint;
  
  std::list< std::pair<Real,int> >::iterator liter = sizes.begin();  
  
  double tsize = factor * ((*liter).first);
  liter++;
  nLevels_=0;
  int elemPerLevel=1;
  distribution_.push_back(elemPerLevel);
  gridSizes_.push_back(tsize);
  
  //greedy cell size heuristic
  //allow objects to be: objectsize = factor * cellsize
  for(;liter!=sizes.end();liter++)
  {
    double dsize=((*liter).first);
    if(dsize > tsize)
    {
      tsize=factor*dsize;
      elemPerLevel=1;
      distribution_.push_back(elemPerLevel);
      gridSizes_.push_back(tsize);
    }
    else
    {
      int &myback = distribution_.back();
      myback++;
    }
  }

  nLevels_=distribution_.size();
      
}

}