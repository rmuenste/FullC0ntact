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

#ifndef COLLIDERFACTORY_H
#define COLLIDERFACTORY_H

#include <rigidbody.h>
#include <collider.h>

namespace i3d {

/**
* @brief A collider factory produces a collider for a contact
*
*
*/
class ColliderFactory
{
public:
	ColliderFactory(void);
  
	~ColliderFactory(void);

 /**
 * Produces a collider for the rigid bodies passed to the function
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */  
	Collider *ProduceCollider(RigidBody *pBody0, RigidBody *pBody1);

private:
  
 /**
 * Produces a collider for a sphere and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
	Collider *CreateColliderSphereX(RigidBody *pBody0, RigidBody *pBody1);
  
 /**
 * Produces a collider for a box and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
	Collider *CreateColliderBoxX(RigidBody *pBody0, RigidBody *pBody1);

 /**
 * Produces a collider for a cylinder and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
	Collider *CreateColliderCylinderX(RigidBody *pBody0, RigidBody *pBody1);

 /**
 * Produces a collider for a plane and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
	Collider *CreateColliderPlaneX(RigidBody *pBody0, RigidBody *pBody1);

 /**
 * Produces a collider for a mesh and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
	Collider *CreateColliderMeshX(RigidBody *pBody0, RigidBody *pBody1);

 /**
 * Produces a collider for a cgal mesh and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
	Collider *CreateColliderCgalMeshX(RigidBody *pBody0, RigidBody *pBody1);

 /**
 * Produces a collider for a box boundary and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
  Collider* CreateColliderBoundaryX(RigidBody *pBody0, RigidBody *pBody1);
  
  /**
 * Produces a collider for a cylinder boundary and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body
 * @return Returns a pointer to the collider
 *
 */
  Collider* CreateColliderCylinderBoundaryX(RigidBody *pBody0, RigidBody *pBody1);

 /**
 * Produces a collider for a compound body and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
  Collider* CreateColliderCompoundX(RigidBody *pBody0, RigidBody *pBody1);  

 /**
 * Produces a collider for a subdomain and an unknown body, this is a different
 * type of collider, because it does not produce contact points, but triggers an
 * event that communicates a body to a different domain, when domain decomposition
 * is used.
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
  Collider* CreateColliderSubDomainX(RigidBody *pBody0, RigidBody *pBody1);  

};

}

#endif

