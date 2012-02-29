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

#ifndef COLLRESPONSELCP_H
#define COLLRESPONSELCP_H
#include <collresponse.h>
#include <matrixnxn.h>
#include <vectorn.h>
#include <lcpsolver.h>
#include <lcpsolvergaussseidel.h>
#include <contactgraph.h>

namespace i3d {

/**
* @brief Collision response module using a LCP formulation of the collision response problem
*/
class CCollResponseLcp : public CCollResponse
{
public:
	/**
	 * Create a new collision response model that uses the LCP formulation
	 * 
	 */
	CCollResponseLcp(void);
	~CCollResponseLcp(void);
	CCollResponseLcp(std::list<CCollisionInfo> *CollInfo, CWorld *pWorld);

	/**
	 * @see CCollResponse::ResolveCollisions()
	 * 
	 */
	void ResolveCollisions();

  void SolveCollidingContact();

  void SolveRestingContact();
  
  void SolveVelocityBased();

  void InitSolverPGS(int maxIterations, Real omega);
  
  void InitSolverPJA(int maxIterations, Real omega);
  
  int GetNumIterations() {return m_pSolver->GetNumIterations();};    

private:
  
	/**
	 * Assembles the LCP for colliding contact. w=Mz+q
	 *
	 * @param M The matrix that is assembled
	 * @param Q The Q vector in the problem formulation
	 * @param vContacts The current contact points for all collisions 
	 * 
	 */
	void AssembleVelocityLcp(CMatrixNxN<double> &M, CVectorN<double> &Q, std::vector<CContact> &vContacts);
  
	/**
	 * Applies the impulses needed to achieve the post-condition for colliding contact
	 * @param nContacts The number of contacts
	 * @param vContacts The current contact points for all collisions 
	 * 
	 */
	void ApplyImpulse(int nContacts, CVectorN<double> &forces, std::vector<CContact> &vContacts);
  
	/**
	 * Assembles the b vector in a = Af + b for resting contact
	 * @param b The b vector in the problem formulation
	 * @param vContacts The current contact points for all collisions 
	 * 
	 */
	void AssembleRhsResting(CVectorN<double> &b, std::vector<CContact> &vContacts);

  /**
   * Assembles the LCP for colliding contact. w=Mz+q such that the post contact 
   * relative velocity in normal direction is 0.
   *
   * @param M The matrix that is assembled
   * @param Q The Q vector in the problem formulation
   * @param vContacts The current contact points for all collisions 
   * 
   */  
  void AssembleVelocityLcp0(CMatrixNxN<double> &M, CVectorN<double> &Q, std::vector<CContact> &vContacts);

  void AssembleVelocityBased(CMatrixNxN<double> &M, CVectorN<double> &Q, std::vector<CContact> &vContacts);

  
  CLcpSolver<Real> *m_pSolver;
  
};

}

#endif // COLLRESPONSELCP_H


