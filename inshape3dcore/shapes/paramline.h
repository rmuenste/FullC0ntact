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


#ifndef CPARAMLINE_H
#define CPARAMLINE_H

#include <shape.h>
#include <segment3.h>

namespace i3d {

template<class T>
class ParamLine : public Shape<T>
{

public:
  ParamLine();
  
  ParamLine(const ParamLine& other);
  
  virtual ~ParamLine();
  
  virtual ParamLine& operator=(const ParamLine& other);

  T getVolume() const {return 0;};

  AABB3<T> getAABB() 
  {
    return AABB3<T>();
  }

  bool isPointInside(const CVector3<T> &vQuery) const
  {
    return false;
  }

  CVector3<T> getCenter() const {return CVector3<T>(0,0,0);};

  std::vector< CVector3<T> > vertices_;
  std::vector< std::pair<int,int> > faces_;
  std::vector< CSegment3<T> > segments_;

};

typedef ParamLine<Real> ParamLiner;

}
#endif // CPARAMLINE_H
