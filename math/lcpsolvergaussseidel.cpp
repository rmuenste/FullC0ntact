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
#include <stdio.h>
#include <iostream>

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
  int i;

  for(iterationsUsed_=0; iterationsUsed_<m_iMaxIterations; ++iterationsUsed_)
  {
    //loop over the rows
    for(i=0; i<n; ++i)
    {
      T inv_aii=1.0;
      delta = 0.0;
      //loop over column
      for(int j(0); j<A.m_iRowPtr[i+1]-A.m_iRowPtr[i]; ++j)
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
    //T dResidual_inf = VectorN<T>::CompMaxNorm(x, x_old);
    //std::cout << "Residual L2: " << m_dResidual << std:endl;
    //if (iterationsUsed_ == 0)
    //{
    //  printf("Initial Residual L2: %E\n", m_dResidual);
    //  printf("Initial Residual L-inf: %E\n", dResidual_inf);
    //  printf("Desired Residual L2: %E\n", m_dResidual*0.0001);
    //}
    //std::cout << "Residual L2: " << std::endl;
    //std::cout << "Residual MaxNorm: " << x.CompMaxNorm() << std:endl;
    //if(m_dResidual < 1e-8 || iterationsUsed_ == m_iMaxIterations-1)
    if (m_dResidual < 1e-8)
    {
      //printf("Final Residual L2: %E\n", m_dResidual);
      //printf("Final Residual L-inf: %E\n", dResidual_inf);
      //A.outputToFile("meshes/matrix.dat");
      //return;
      break;
    }
  }
  //printf("Final Residual L2: %E\n", m_dResidual);
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class LcpSolverGaussSeidel<Real>;

//----------------------------------------------------------------------------
}
