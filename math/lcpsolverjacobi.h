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
class CLcpSolverJacobi : public CLcpSolver<T> {

public: 

  CLcpSolverJacobi();
  
  CLcpSolverJacobi(int maxIter, T omega) : CLcpSolver<T>(maxIter), m_dOmega(omega) {};   
  
  ~CLcpSolverJacobi(); 

  void SetMatrix(CMatrixNxN<T> &M){m_matM=&M;};

  void SetQWZ(CVectorN<T> &Q,CVectorN<T> &W,CVectorN<T> &Z){m_vQ=&Q;m_vW=&W;m_vZ=&Z;};

  void Solve();
  
  void CleanUp() {m_matM=NULL;m_vQ=NULL;m_vW=NULL;m_vZ=NULL;}; 
  
  int GetNumIterations() {return m_iIterationsUsed;};  

  T   m_dOmega;
  
  CMatrixNxN<T> *m_matM;
  CVectorN<T>   *m_vQ;
  CVectorN<T>   *m_vW;
  CVectorN<T>   *m_vZ;  
	
	using CLcpSolver<T>::m_iMaxIterations;
	
	using CLcpSolver<T>::m_iIterationsUsed;

	using CLcpSolver<T>::m_dResidual;	
  
  
  
};

}

#endif
