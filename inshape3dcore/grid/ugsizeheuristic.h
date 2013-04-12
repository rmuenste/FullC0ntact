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

class CUGDescriptor 
{
public:
  
  CUGDescriptor();
  
  CUGDescriptor(int levels, std::vector<int> &dist, std::vector<Real> &sizes)
  {
    m_iLevels = levels;
    m_vDistribution = dist;
    m_vGridSizes = sizes;
  }
  
  CUGDescriptor(const CUGDescriptor& other)
  {
    m_iLevels = other.m_iLevels;
    m_vDistribution = other.m_vDistribution;
    m_vGridSizes = other.m_vGridSizes;    
  }
  
  // levels generated
  int m_iLevels;

  //cell size distribution
  std::vector<int> m_vDistribution;

  //sizes of grid levels
  std::vector<Real> m_vGridSizes; 
};
  
class CUGSizeHeuristic
{

public:
  CUGSizeHeuristic();
  
  CUGSizeHeuristic(const CUGSizeHeuristic& other);

  virtual void ComputeCellSizes(std::list< std::pair<Real,int> > &sizes, Real sizeHint);

  virtual ~CUGSizeHeuristic();
  
  CUGDescriptor GetDescriptor();

  // levels generated
  int m_iLevels;

  //cell size distribution
  std::vector<int> m_vDistribution;

  //sizes of grid levels
  std::vector<Real> m_vGridSizes;

};



}
#endif // UGSIZEHEURISTIC_H

