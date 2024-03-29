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


#include "paramline.h"

namespace i3d {

template<class T>
ParamLine<T>::ParamLine() : center_(0,0,0)
{

}

template<class T>
ParamLine<T>::ParamLine(const ParamLine& other)
{

}

template<class T>
ParamLine<T>::~ParamLine()
{

}

template<class T>
ParamLine<T>& ParamLine<T>::operator=(const ParamLine& other)
{
return *this;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class ParamLine<Real>;
//----------------------------------------------------------------------------




}
