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

#ifndef SUBDOMAINBOUNDARY_H
#define SUBDOMAINBOUNDARY_H

#include <compoundbody.h>

namespace i3d {

/**
*  @brief A body that is a non-physical boundary, but a
*         boundary between computational domains, in case
*         a domain decomposition approach is used.
*
*  A body that is compounded of several rigid bodies 
*/
class CSubdomainBoundary : public CCompoundBody
{
public:

  /**
  *
  * Creates an empty rigid body
  *
  */
  CSubdomainBoundary();

  ~CSubdomainBoundary();

  /** 
  * Copy a rigid body
  */
  CSubdomainBoundary(const CSubdomainBoundary& copy);

  void TranslateTo(const VECTOR3 &vPos);
  
  /** 
  *
  * Computes the inverse of the inertia tensor and stores it
  * in the member variable m_InvInertiaTensor
  *
  */
  void GenerateInvInertiaTensor();

  /**
   * Returns the radius of a bounding sphere for the body
   **/
  Real GetBoundingSphereRadius() {return 1.0;}

  /**
  * @see CRigidBody::GetID
  */
  int GetID() {return m_iID;};

  /**
  * @see CRigidBody::SetID
  */
  void SetID(int id)
  {
    m_iID=id;
    std::vector<CRigidBody*>::iterator iter = m_pBodies.begin();
    for(;iter!=m_pBodies.end();iter++)
    {
      (*iter)->SetID(id);
    }
  };

  /**
  * Get the number of bodies that form the compound body
  */
  inline unsigned int GetNumComponents() {return m_pBodies.size();};

  /**
  * Get the number of bodies that form the compound body
  */
  inline CRigidBody* GetComponent(int i){return m_pBodies[i];};
  
  /**
  * Returns the neighbor at the i-th boundary component
  */
  inline int GetNeighbor(int i) {return m_iNeighbors[i];};
  
  /**
  * Set the neighbor at the i-th boundary component
  */
  inline void SetNeighbor(int i, int j) {m_iNeighbors[i]=j;};  

  /**
  * Returns the number of neighbors
  */
  inline int GetNumNeighbors() {return m_iNumNeighbors;};
  
  /**
  * Sets the number of neighbors
  */
  inline void SetNumNeighbors(int i) {m_iNumNeighbors=i;};  

  /**
   * Returns the maximum number of remotes in the neighbors group
   */
  inline int GetMaxRemotes() {
    int maxRemotes = m_iRemoteBodies[0].size();
    for(int i=1;i<26;i++)
    {
      if(maxRemotes < m_iRemoteBodies[i].size())
        maxRemotes=m_iRemoteBodies[i].size();
    }
    return maxRemotes;
  };

  /**
   * Returns a pointer to a vector of
   */

    
  /**
  * The mpi rank (with regard to the MPI_COMM_WORLD group) of the neighbors at the different boundary components
  */
  int m_iNeighbors[26];
  
  /**
   * The actual number of neighbors
   */
  int m_iNumNeighbors;

  /**
   * Maps the ranks of our root group to the geometric neighbors
   */
  int m_iMapGroupNeighbors[26];

  /**
   * Maps the geometric neighbors to the ranks of the root group
   */
  int m_iMapNeighborsGroup[26];
  
  /**
   * The ids of our remote bodies in the remote m_vRigidBodies vector
   */
  std::vector<int> m_iRemoteIDs[26];
  
  /**
   * The indices of the remote bodies in the local m_vRigidBodies vector
   */ 
  std::vector<int> m_iRemoteBodies[26];
  
  /**
   * A buffer that stores the bodies that are scheduled to be send
   */
   //std::list<int> m_lSendList[26];

};

}

#endif // RIGIDBODY_H
