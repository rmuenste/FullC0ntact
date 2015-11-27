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
LcpSolverGaussSeidel<T>::LcpSolverGaussSeidel() 
{

}

template <class T>
LcpSolverGaussSeidel<T>::~LcpSolverGaussSeidel() 
{

}

template <class T>
void LcpSolverGaussSeidel<T>::Solve()
{
  
  //MatrixNxN<T> &A=*m_matM;
  MatrixCSR<T> &A = *m_matMCSR;

  VectorN<T>   &b=*m_vQ;
  VectorN<T>   &x=*m_vZ;    
  int n = A.m_iN;
  VectorN<T>   x_old(n);

  T delta;
  int i,iter;

  for(iter=0;iter<m_iMaxIterations;iter++)
  {
    //loop over the rows
    for(i=0;i<n;i++)
    {
      T inv_aii=1.0;
      delta = 0.0;
      //loop over column
      for(int j=0;j<A.m_iRowPtr[i+1]-A.m_iRowPtr[i];j++)
      {
        //get the index into the values array
        int index = A.m_iRowPtr[i]+j;

        //get the column index
        int col = A.m_iColInd[index];

        //if this is the diagonal element
        if(col==i)
          inv_aii/=A.m_dValues[index];
        else
        { 
          delta += A.m_dValues[index] * x(col);
        }
      }
      
      //compute the update
      delta=inv_aii*(b(i)-delta);

      //backup the old solution
      x_old(i)=x(i);
      
      //update the solution
      x(i)=x(i) + m_dOmega*(delta - x(i));
      
      //projection step
      if(x(i) < 0.0)x(i)=0.0;    
    }
    //check the residual
    m_dResidual = VectorN<T>::CompNorm(x,x_old);
    if(m_dResidual < 1e-8)
    {
      m_iIterationsUsed=iter;
      return;
    }
  }


  //for(iter=0;iter<m_iMaxIterations;iter++)
  //{
  //  for(i=0;i<n;i++)
  //  {
  //    T inv_aii=1.0/A(i,i);
  //    delta=0.0;
  //    for(j=0;j<i;j++)
  //      delta+=A(i,j)*x(j);
  //    for(j=i+1;j<n;j++)
  //      delta+=A(i,j)*x(j);

  //    delta=inv_aii*(b(i)-delta);

  //    x_old(i)=x(i);
  //    
  //    x(i)=x(i) + m_dOmega*(delta - x(i));
  //    
  //    if(x(i) < 0.0)x(i)=0.0;
  //  }

  //  //check the residual
  //  m_dResidual = VectorN<T>::CompNorm(x,x_old);
  //  if(m_dResidual < 1e-8)
  //  {
  //    m_iIterationsUsed=iter;
  //    return;
  //  }
  //}

}

//template <class T>
//void LcpSolverGaussSeidel<T>::Solve()
//{
//  
//  MatrixNxN<T> &A=*m_matM;
//
//  VectorN<T>   &b=*m_vQ;
//  VectorN<T>   &x=*m_vZ;    
//  int n = A.m_iN;
//  VectorN<T>   x_old(n);
//
//  T delta;
//  int i,j,iter;
//
//  for(iter=0;iter<m_iMaxIterations;iter++)
//  {
//    for(i=0;i<n;i++)
//    {
//      T inv_aii=1.0/A(i,i);
//      delta=0.0;
//      for(j=0;j<i;j++)
//        delta+=A(i,j)*x(j);
//      for(j=i+1;j<n;j++)
//        delta+=A(i,j)*x(j);
//
//      delta=inv_aii*(b(i)-delta);
//
//      x_old(i)=x(i);
//      
//      x(i)=x(i) + m_dOmega*(delta - x(i));
//      
//      if(x(i) < 0.0)x(i)=0.0;
//    }
//
//    //check the residual
//    m_dResidual = VectorN<T>::CompNorm(x,x_old);
//    if(m_dResidual < 1e-8)
//    {
//      m_iIterationsUsed=iter;
//      return;
//    }
//  }
//
//}
  

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class LcpSolverGaussSeidel<Real>;

//----------------------------------------------------------------------------
}
