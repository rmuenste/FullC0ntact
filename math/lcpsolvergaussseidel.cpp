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


//===================================================
//                     INCLUDES
//===================================================


#include "lcpsolvergaussseidel.h"

namespace i3d {

template <class T>
CLcpSolverGaussSeidel<T>::CLcpSolverGaussSeidel() 
{

}

template <class T>
CLcpSolverGaussSeidel<T>::~CLcpSolverGaussSeidel() 
{

}

template <class T>
void CLcpSolverGaussSeidel<T>::Solve()
{
  
  CMatrixNxN<T> &A=*m_matM;
  CVectorN<T>   &b=*m_vQ;
  CVectorN<T>   &x=*m_vZ;    
  int n = A.rows();
  CVectorN<T>   x_old(n);

  T delta;
  int i,j,iter;

  for(iter=0;iter<m_iMaxIterations;iter++)
  {
    for(i=0;i<n;i++)
    {
      T inv_aii=1.0/A(i,i);
      delta=0.0;
      for(j=0;j<i;j++)
        delta+=A(i,j)*x(j);
      for(j=i+1;j<n;j++)
        delta+=A(i,j)*x(j);

      delta=inv_aii*(b(i)-delta);

      x_old(i)=x(i);
      
      x(i)=x(i) + m_dOmega*(delta - x(i));
      
      if(x(i) < 0.0)x(i)=0.0;
    }

    //check the residual
    m_dResidual = CVectorN<T>::CompNorm(x,x_old);
    if(m_dResidual < 1e-8)
    {
      m_iIterationsUsed=iter;
      return;
    }
  }

}
  

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CLcpSolverGaussSeidel<Real>;

//----------------------------------------------------------------------------
}