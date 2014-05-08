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
#ifndef LCPSOLVERJACOBI_H
#define LCPSOLVERJACOBI_H



//===================================================
//                     INCLUDES
//===================================================
#include <vectorn.h>
#include <matrixnxn.h>
#include <lcpsolver.h>
#include <stdlib.h>
#include <stdio.h>

namespace i3d {

/**
* @brief A LCP solver using the iterative projected jacobi method
*
* A iterative LCP solver that solves
* the LCP w = Mz + q, w o z = 0, w >= 0, z >= 0.
*/  
template <class T>
class LcpSolverJacobi : public LcpSolver<T> {

public: 

  LcpSolverJacobi();
  
  LcpSolverJacobi(int maxIter, T omega) : LcpSolver<T>(maxIter), m_dOmega(omega) {};   
  
  ~LcpSolverJacobi(); 

  void SetMatrix(MatrixNxN<T> &M){m_matM=&M;};

  void SetMatrix(MatrixCSR<T> &M){};

  void SetQWZ(VectorN<T> &Q,VectorN<T> &W,VectorN<T> &Z){m_vQ=&Q;m_vW=&W;m_vZ=&Z;};

  void Solve();
  
  void CleanUp() {m_matM=NULL;m_vQ=NULL;m_vW=NULL;m_vZ=NULL;}; 
  
  int GetNumIterations() {return m_iIterationsUsed;};  

  T   m_dOmega;
  
  MatrixNxN<T> *m_matM;
  VectorN<T>   *m_vQ;
  VectorN<T>   *m_vW;
  VectorN<T>   *m_vZ;  
	
	using LcpSolver<T>::m_iMaxIterations;
	
	using LcpSolver<T>::m_iIterationsUsed;

	using LcpSolver<T>::m_dResidual;	
  
  
  
};

}

#endif
