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
class CColliderFactory
{
public:
	CColliderFactory(void);
  
	~CColliderFactory(void);

 /**
 * Produces a collider for the rigid bodies passed to the function
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */  
	CCollider *ProduceCollider(CRigidBody *pBody0, CRigidBody *pBody1);

private:
  
 /**
 * Produces a collider for a sphere and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
	CCollider *CreateColliderSphereX(CRigidBody *pBody0, CRigidBody *pBody1);
  
 /**
 * Produces a collider for a box and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
	CCollider *CreateColliderBoxX(CRigidBody *pBody0, CRigidBody *pBody1);

 /**
 * Produces a collider for a cylinder and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
	CCollider *CreateColliderCylinderX(CRigidBody *pBody0, CRigidBody *pBody1);

 /**
 * Produces a collider for a plane and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
	CCollider *CreateColliderPlaneX(CRigidBody *pBody0, CRigidBody *pBody1);

 /**
 * Produces a collider for a mesh and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
	CCollider *CreateColliderMeshX(CRigidBody *pBody0, CRigidBody *pBody1);

 /**
 * Produces a collider for a mesh and a yet unknown shape
 *
 * @param pBody0 The first body
 * @param pBody1 The second body 
 * @return Returns a pointer to the collider
 *
 */    
  CCollider* CreateColliderBoundaryX(CRigidBody *pBody0, CRigidBody *pBody1);


};

}

#endif

