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

#include "subdomainboundary.h"
#include <sphere.h>
#include <boundarybox.h>
#include <rigidbodyio.h>
#include <stdlib.h>
#include <cylinder.h>
#include <meshobject.h>
#include <genericloader.h>
#include <3dmodel.h>
#include <subdivisioncreator.h>
#include <collisioninfo.h>

namespace i3d {

SubdomainBoundary::SubdomainBoundary() : CompoundBody()
{

}

SubdomainBoundary::~SubdomainBoundary()
{

}

SubdomainBoundary::SubdomainBoundary(const i3d::SubdomainBoundary& copy) : CompoundBody(copy)
{

}

void SubdomainBoundary::translateTo(const VECTOR3& vPos)
{
    i3d::RigidBody::translateTo(vPos);
}
  
void SubdomainBoundary::generateInvInertiaTensor()
{
  invInertiaTensor_.SetZero();
}
  
}


