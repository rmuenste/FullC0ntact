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


#ifndef UGSIZEHEURISTIC_H
#define UGSIZEHEURISTIC_H

#include <vector>
#include <list>
#include <mymath.h>

namespace i3d {

class UniformGridDescriptor 
{
public:
  
  UniformGridDescriptor();
  
  UniformGridDescriptor(int levels, std::vector<int> &dist, std::vector<Real> &sizes)
  {
    nLevels_      = levels;
    distribution_ = dist;
    gridSizes_    = sizes;
  }
  
  UniformGridDescriptor(const UniformGridDescriptor& other)
  {
    nLevels_      = other.nLevels_;
    distribution_ = other.distribution_;
    gridSizes_    = other.gridSizes_;    
  }
  
  // levels generated
  int nLevels_;

  //cell size distribution
  std::vector<int> distribution_;

  //sizes of grid levels
  std::vector<Real> gridSizes_; 
};
  
class UniformGridSizeHeuristic
{

public:
  
  UniformGridSizeHeuristic();
  
  UniformGridSizeHeuristic(const UniformGridSizeHeuristic& other);

  virtual void computeCellSizes(std::list< std::pair<Real,int> > &sizes, Real sizeHint);

  virtual ~UniformGridSizeHeuristic();
  
  UniformGridDescriptor getDescriptor();

  // levels generated
  int nLevels_;

  //cell size distribution
  std::vector<int> distribution_;

  //sizes of grid levels
  std::vector<Real> gridSizes_;

};



}
#endif // UGSIZEHEURISTIC_H

