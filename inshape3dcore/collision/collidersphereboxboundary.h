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

#ifndef COLLIDERSPHEREBOXBOUNDARY_H
#define COLLIDERSPHEREBOXBOUNDARY_H

#include <rigidbody.h>
#include <contact.h>
#include <boundarybox.h>
#include <collider.h>

namespace i3d {

/**
* @brief A Collider for a sphere and a box-shaped boundary
*
*/
class CColliderSphereBoxBoundary : public CCollider
{
public:
  
	CColliderSphereBoxBoundary(void);
  
	~CColliderSphereBoxBoundary(void);

 /**
 * @see CCollider::Collide
 *
 */  
	void Collide(std::vector<CContact> &vContacts);

};

}

#endif
