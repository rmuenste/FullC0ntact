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

#include "segment3.h"

namespace i3d {

template <typename T>
CSegment3<T>::CSegment3(void)
{
}

template <typename T>
CSegment3<T>::CSegment3(const CSegment3<T> &copy)
{
	this->m_Ext=copy.m_Ext;
	this->m_vDir = copy.m_vDir;
	this->m_vP0 = copy.m_vP0;
	this->m_vP1 = copy.m_vP1;
	this->m_vCenter = copy.m_vCenter;
}


template <typename T>
CSegment3<T>::~CSegment3(void)
{
}

template <typename T>
CSegment3<T>::CSegment3(const CVector3<T> &vOrig, const CVector3<T> &vDir, T ext)
{
	m_vCenter = vOrig;
	m_vDir    = vDir;
	m_Ext     = ext;
	CalcVertices();
}

template <typename T>
CSegment3<T>::CSegment3(const CVector3<T> &vP0, const CVector3<T> &vP1)
{
	m_vP0 = vP0;
	m_vP1 = vP1;
	CalcExtent();
}

template <typename T>
void CSegment3<T>::CalcExtent(void)
{
	m_vCenter = ((T)0.5)*(m_vP0 + m_vP1);
	m_vDir = m_vP1 - m_vP0;
	m_Ext = ((T)0.5)*m_vDir.mag();
	m_vDir.Normalize();
}

template <typename T>
void CSegment3<T>::CalcVertices(void)
{
	m_vP0=m_vCenter - m_vDir * m_Ext;
	m_vP1=m_vCenter + m_vDir * m_Ext;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CSegment3<float>;

template class CSegment3<double>;
//----------------------------------------------------------------------------

}