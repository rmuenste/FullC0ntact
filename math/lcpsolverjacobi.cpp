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


#include "lcpsolverjacobi.h"

namespace i3d {

template <class T>
LcpSolverJacobi<T>::LcpSolverJacobi() 
{

}

template <class T>
LcpSolverJacobi<T>::~LcpSolverJacobi() 
{

}

template <class T>
void LcpSolverJacobi<T>::Solve()
{
  
  MatrixNxN<T> &A=*m_matM;
  VectorN<T>   &b=*m_vQ;
  VectorN<T>   &x=*m_vZ;    
  int n = A.rows();
  VectorN<T>   x_old(n);
  VectorN<T>   y(n);  

  T delta;
  int i,j;

  for(iterationsUsed_=0; iterationsUsed_<m_iMaxIterations; ++iterationsUsed_)
  {
    for(i=0;i<n;i++)
    {
      T inv_aii=1.0/A(i,i);
      delta=0.0;
      for(j=0;j<i;j++)
        delta+=A(i,j)*x(j);
      for(j=i+1;j<n;j++)
        delta+=A(i,j)*x(j);

      //x_old(i)=x(i);
      
      y(i)= inv_aii*(b(i)-delta);
      
      if(y(i) < 0.0)y(i)=0.0;
    }
    
    x_old = x;
    x     = y;

    //check the residual
    m_dResidual = VectorN<T>::CompNorm(x,x_old);
    if(m_dResidual < 1e-8)
    {
      return;
    }
  }

}
  

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class LcpSolverJacobi<Real>;

//----------------------------------------------------------------------------
}
