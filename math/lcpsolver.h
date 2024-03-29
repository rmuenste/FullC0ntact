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
#ifndef LCPSOLVER_H
#define LCPSOLVER_H



//===================================================
//                     INCLUDES
//===================================================
#include <matrixnxn.h>
#include <vectorn.h>
#include <mymath.h>
#include <matrixcsr.h>

namespace i3d {

/**
* @brief The basic Lcp solver interface
*
* This abstract class defines the basic lcp solver interface
*/
template <class T>
class LcpSolver {

public: 

  LcpSolver();     

  LcpSolver(int maxIter) : m_iMaxIterations(maxIter), iterationsUsed_(0) {} ;     

  virtual ~LcpSolver(); 

  virtual void SetMatrix(MatrixNxN<T> &M)=0;

  virtual void SetMatrix(MatrixCSR<T> &M)=0;

  virtual void SetQWZ(VectorN<T> &Q,VectorN<T> &W,VectorN<T> &Z)=0;

  virtual void Solve()=0;
  
  virtual void CleanUp()=0;
  
  int GetNumIterations() {return iterationsUsed_;};  
  
  int m_iMaxIterations;
  int iterationsUsed_;
  T   m_dResidual;


};

}

#endif
