/*
    <Class to store information about a contact group.>
    Copyright (C) 2011  <Raphael Muenster> <email>

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


#ifndef CCONTACTGROUP_H
#define CCONTACTGROUP_H

//===================================================
//                     INCLUDES
//===================================================
#include<rigidbody.h>
#include<collisioninfo.h>
#include<stacklayer.h>

namespace i3d {

/**
 * @brief A contact group is a collection of particles that corresponds to a connected component in the contact graph
 * 
 */
  
class ContactGroup
{

public:
    ContactGroup();
    ContactGroup(int id) : m_iGroupId(id)
    {
      m_pLayers = NULL;
      m_iMaxHeight = 0;
      m_iLayers    = 0;      
    };    
    ContactGroup(const ContactGroup& other);
    virtual ~ContactGroup();

    std::list<RigidBody*>     m_pBodies;
    std::list<CollisionInfo*> m_pEdges;
    StackLayer*               m_pLayers;
    
    int m_iMaxHeight;
    int m_iGroupId;
    int m_iLayers;
    
};

}
#endif // CCONTACTGROUP_H
