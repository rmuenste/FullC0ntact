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

#include "meshobject.h"
#include <genericloader.h>
#include <vector3.h>


namespace i3d {

template<class T>
CMeshObject<T>::CMeshObject(void )
{

}

template<class T>
CMeshObject<T>::~CMeshObject(void )
{

}

template<class T>
T CMeshObject<T>::Volume() const
{
    //TODO:implement
    //ATM returns standard mesh volume of torus
    //2*pi^2 * r_xy*r_xz^2
  //return T(2.0*CMath<T>::SYS_PI * 0.1 * 0.01*0.01);
  T vol = T(8.24013233e-4);
  return vol;
}

template<class T>
CMeshObject<T>::CMeshObject(const char* strFilename)
{
	CGenericLoader loader;
	loader.ReadModelFromFile(&m_Model,strFilename);
	m_Model.GenerateBoundingBox();
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CMeshObject<Real>;
//----------------------------------------------------------------------------

}