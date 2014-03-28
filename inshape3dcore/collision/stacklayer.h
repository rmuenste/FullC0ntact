/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

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
#ifndef STACKLAYER_H
#define STACKLAYER_H



//===================================================
//                     INCLUDES
//===================================================
#include <rigidbody.h>
#include <collisioninfo.h>

namespace i3d {

/**
 * @brief A class that stores rigid bodies that are in a special contact configuration called 'stack layer'.
 * 
 * Imagine 3 boxes on the ground and we stack another 3 boxes on top of them. Then we have a stack of boxes.
 * The lower 3 boxes are a stack layer and the upper 3 boxes are a stack layer.
 */  
class StackLayer {

public: 

StackLayer(); 

~StackLayer(); 

void AddBody(RigidBody *pBody) {m_pBodies.push_back(pBody);};

void AddEdge(CollisionInfo *info) {m_pEdges.push_back(info);};

std::list<RigidBody*>     m_pBodies;
std::list<CollisionInfo*> m_pEdges;

};

}
#endif
