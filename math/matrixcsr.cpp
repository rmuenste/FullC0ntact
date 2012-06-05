#include <stdio.h>
#include <iostream>
#include <string.h>
#include "matrixcsr.h"

namespace i3d {

template<class T>
CMatrixCSR<T>::CMatrixCSR(void)
{
  m_dValues = NULL;
  m_iColInd = NULL;
  m_iRowPtr = NULL;
}

template<class T>
CMatrixCSR<T>::~CMatrixCSR(void)
{
  if(m_dValues != NULL)
  {
    delete[] m_dValues;
    m_dValues = NULL;
  }
  if(m_iRowPtr != NULL)
  {
    delete[] m_iRowPtr;
    m_iRowPtr = NULL;
  }
  if(m_iColInd != NULL)
  {
    delete[] m_iColInd;
    m_iColInd = NULL;
  }
}
	
template<class T>
CMatrixCSR<T>::CMatrixCSR(const CMatrixNxN<T> &matrix)
{
  int entries = matrix.m_iN * matrix.m_iN;
  entries     = entries - matrix.NumZeros();
  m_dValues = new T[entries];
  m_iColInd = new int[entries];
  m_iRowPtr = new int[matrix.m_iN+1];
  m_iN = matrix.m_iN;
  m_iNumVal = entries;
  int index = 0;
  m_iRowPtr[0] = 0;


  //for every row
  for(int i=0;i<matrix.m_iN;i++)
  {
    int EntriesInRow = 0;
    //iterate through the columns of the row
    for(int j=0;j<matrix.m_iN;j++)
    {
      if(matrix(i,j)!=0.0)
      {
        m_dValues[index] = matrix(i,j);
        m_iColInd[index] = j;
        index++;
        EntriesInRow++;
      }
    }
    m_iRowPtr[i+1]=m_iRowPtr[i]+EntriesInRow;
  }

}

template<class T>
CMatrixCSR<T>::CMatrixCSR(int n)
{
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CMatrixCSR<float>;

template class CMatrixCSR<double>;
//----------------------------------------------------------------------------

}