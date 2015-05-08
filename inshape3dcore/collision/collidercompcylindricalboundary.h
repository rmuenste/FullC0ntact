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


#ifndef COLLIDERCOMPCYLINDRICALBOUNDARY_H
#define COLLIDERCOMPCYLINDRICALBOUNDARY_H

#include <collider.h>

namespace i3d {

/**
* @brief A Collider for a sphere and a cylindrical boundary
*
*/
class ColliderCompCylindricalBoundary : public Collider
{
public:

  ColliderCompCylindricalBoundary(void);

  ~ColliderCompCylindricalBoundary(void);

  /**
  * @see CCollider::Collide
  */
  void collide(std::vector<Contact> &vContacts);

private:
  void collideHollow(std::vector<Contact> &vContacts);

};

}

#endif // COLLIDERCOMPCYLINDRICALBOUNDARY_H
