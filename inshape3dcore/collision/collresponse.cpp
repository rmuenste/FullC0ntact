/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#include "collresponse.h"
#include <world.h>

namespace i3d {

CollResponse::CollResponse() :
m_pWorld(nullptr),
m_iContactPoints(0),
m_pGraph(nullptr),
dTimeAssembly(0.0),
dTimeSolver(0.0),
dTimeSolverPost(0.0),
dTimeAssemblyDry(0.0)
{

  //std::list<CollisionInfo> *m_CollInfo;
  ///** If the distance between two objects is below m_dCollEps they are declared as collided */
  //Real m_dCollEps;

}


CollResponse::CollResponse(std::list<CollisionInfo> *CollInfo, World *pWorld) : 
  m_CollInfo(CollInfo),
  m_pWorld(pWorld)
{

}


void CollResponse::Solve()
{

}

CollResponse::~CollResponse()
{

}

}
