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
#include <matrixcsr.h>

namespace i3d {

/**
* @brief Collision response module using a LCP formulation of the collision response problem
*/
class CollResponseLcp : public CollResponse
{
public:
	/**
	 * Create a new collision response model that uses the LCP formulation
	 * 
	 */
	CollResponseLcp(void);
	~CollResponseLcp(void);
	CollResponseLcp(std::list<CollisionInfo> *CollInfo, World *pWorld);

	/**
	 * @see CCollResponse::Solve()
	 * 
	 */
	void Solve();

  void InitSolverPGS(int maxIterations, Real omega);
  
  void InitSolverPJA(int maxIterations, Real omega);
  
  int GetNumIterations() {return m_pSolver->GetNumIterations();};    

private:
    
  /**
    * Applies the impulses needed to achieve the post-condition for colliding contact
    * @param nContacts The number of contacts
    * @param vContacts The current contact points for all collisions 
    * 
    */
  void ApplyImpulse(int nContacts, VectorN<double> &forces, std::vector<Contact*> &vContacts);
  
  /**
    * Assembles the matrix for the velocity-based LCP formulation of the contact problem.
    * w=Mz+q
    *
    * @param M The matrix that is assembled
    * @param Q The Q vector in the problem formulation
    * @param vContacts The current contact points for all collisions 
    * 
    */
  void AssembleVelocityBased(MatrixNxN<double> &M, VectorN<double> &Q, std::vector<Contact*> &vContacts);

  /**
    * Assembles a csr matrix for the velocity-based LCP formulation of the contact problem.
    * w=Mz+q
    *
    * @param M The matrix that is assembled in CSR format
    * @param Q The Q vector in the problem formulation
    * @param vContacts The current contact points for all collisions
    *
    */
  void AssembleVelocityBasedCSR(MatrixCSR<double> &M, VectorN<double> &Q, std::vector<Contact*> &vContacts);

  void AssembleVelocityBasedCSRGraph(MatrixCSR<double> &M, VectorN<double> &Q, std::vector<Contact*> &vContacts);

  int ComputeMatrixStructure(std::vector<Contact*> &vContacts, int *rowPointer);  

  int ComputeMatrixStructureGraph(std::vector<Contact*> &vContacts, int *rowPointer);
  
  void ComputeTangentSpace(const VECTOR3& normal, VECTOR3& t1, VECTOR3& t2);
  
  LcpSolver<Real> *m_pSolver;
  
};

}

#endif // COLLRESPONSELCP_H


