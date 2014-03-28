/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2011  <copyright holder> <email>

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


#include "contactgroup.h"

namespace i3d {

ContactGroup::ContactGroup()
{
  m_pLayers = NULL;
  m_iGroupId   = -2;
  m_iMaxHeight = 0;
  m_iLayers    = 0;        
}

ContactGroup::ContactGroup(const ContactGroup& other)
{
  m_pBodies    = other.m_pBodies;
  m_pEdges     = other.m_pEdges;
  m_iGroupId   = other.m_iGroupId;
  m_iMaxHeight = other.m_iMaxHeight;
  m_iLayers    = other.m_iLayers;
  
  if(m_iLayers != 0)
  {
    m_pLayers = new StackLayer[m_iLayers];
    for(int i=0;i<m_iLayers;i++)
    {
      m_pLayers[i]=other.m_pLayers[i];
    }
  }
  else
  {
    m_pLayers = NULL;
  }
}

ContactGroup::~ContactGroup()
{
  if(m_pLayers != NULL)
  {
    delete[] m_pLayers;
    m_pLayers = NULL;
  }
}

}
