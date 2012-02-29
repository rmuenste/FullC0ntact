/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#include "pathsolver.h"
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <iostream>
#include <Types.h>

extern "C" void SimpleLCP(int variables,
               int m_nnz, int *m_i, int *m_j, double *m_ij, double *q,
	             double *lb, double *ub,
               MCP_Termination *status, double *z);

namespace i3d {

CPathSolver::CPathSolver()
{
  m_i = NULL;
  m_ij= NULL;
  m_lb= NULL;
  m_j = NULL;
  m_ub= NULL;
  m_q = NULL;
  m_z = NULL;
}

CPathSolver::~CPathSolver()
{
  
  if(m_i != NULL)
  {
    delete[]m_i;
    m_i=NULL;
  }
  if(m_ij != NULL)
  {
    delete[]m_ij;
    m_ij=NULL;
  }
  if(m_lb != NULL)
  {
    delete[]m_lb;
    m_lb=NULL;
  }
  if(m_j != NULL)
  {
    delete[]m_j;
    m_j=NULL;
  }
  if(m_ub != NULL)
  {
    delete[]m_ub;
    m_ub=NULL;
  }
  if(m_q != NULL)
  {
    delete[]m_q;
    m_q=NULL;
  }
  if(m_z != NULL)
  {
    delete[]m_z;
    m_z=NULL;
  }

}

CPathSolver::CPathSolver (int numEquations, double** M, double* Q, double* Z,
    double* W, int& status, int maxRetries, double zeroTolerance,
    double ratioError)
    :
    mNumEquations(numEquations),
    mM(M),
    mQ(Q),
    mZ(Z),
    mMaxRetries(maxRetries),
    mZeroTolerance(zeroTolerance),
    mRatioError(ratioError)
{

  //compute number of non-zeros
  ComputeNonZeroEntries();

  //allocate the arrays
  m_i = new int[m_nnz+1];
  m_j = new int[m_nnz+1];
  m_ij = new double[m_nnz+1];
  m_lb = new double[m_nnz+1];
  m_ub = new double[m_nnz+1];
  m_q = new double[m_nnz+1];
  m_z = new double[m_nnz+1];

  //convert the data to path format
  GeneratePathFormat();

  //set the lower and upper bounds
  for(int i=0;i<mNumEquations;i++)
  {
    m_lb[i]=-mZeroTolerance;
    m_ub[i]=1e20;
    m_z[i]=0.0;
    m_q[i]=mQ[i];
  }

}

void CPathSolver::Solve()
{
#ifdef _MSC_VER  
  MCP_Termination termination;
  SimpleLCP(mNumEquations,m_nnz,m_i,m_j,m_ij,m_q,m_lb,m_ub,&termination,m_z);

  if(termination==MCP_Error)
  {
    std::cerr<<"PATH: Error in the solution"<<std::endl;
  }
  else if(termination==MCP_Solved)
  {
    for(int i=0;i<mNumEquations;i++)
    {
      mZ[i]=m_z[i];
    }
  }
  else
  {
    std::cerr<<"PATH: Other error."<<std::endl;
  }
#endif
}

void CPathSolver::ComputeNonZeroEntries()
{
  m_nnz=0;
  for(int i=0;i<mNumEquations;i++)
  {
    for(int j=0;j<mNumEquations;j++)
    {
      if(fabs(mM[i][j])>mZeroTolerance)
      {
        m_nnz++;
      }
    }
  }
}

void CPathSolver::GeneratePathFormat()
{
  int index = 0;
  for(int j=0;j<mNumEquations;j++)
  {
    for(int i=0;i<mNumEquations;i++)
    {
      if(fabs(mM[i][j])>mZeroTolerance)
      {
        m_i[index]=i+1;
        m_j[index]=j+1;
        m_ij[index]=mM[i][j];
        index++;
      }
    }
  }
}

}