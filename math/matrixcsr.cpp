#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include "matrixcsr.h"

namespace i3d {

template<class T>
MatrixCSR<T>::MatrixCSR(void)
{
  m_dValues = NULL;
  m_iColInd = NULL;
  m_iRowPtr = NULL;
}

template<class T>
MatrixCSR<T>::~MatrixCSR(void)
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
MatrixCSR<T>::MatrixCSR(int n, int entries, int *rowPointer)
{
  m_dValues = new T[entries];
  m_iColInd = new int[entries];
  m_iRowPtr = rowPointer;
  m_iN = n;
  m_iNumVal = entries;
}

template<class T>
MatrixCSR<T>::MatrixCSR(const MatrixNxN<T> &matrix)
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
void MatrixCSR<T>::outputToFile(std::string fileName)
{

  using namespace std;
  ofstream myfile(fileName);

  //check
  if (!myfile.is_open())
  {
    cout << "Error opening file: " << fileName << endl;
    std::exit(EXIT_FAILURE);
  }//end if

  for (int i(0); i<m_iN; ++i)
  {
    for (int j(m_iRowPtr[i]); j<m_iRowPtr[i + 1]; ++j)
    {
      int row_index = i + 1;
      int col_index = m_iColInd[j] + 1;
      
      myfile << row_index << " " << col_index << " " << m_dValues[j] << "\n";

      //if (m_dValues[j] < 0)
      //{
      //  std::cout << m_dValues[j] << " ";
      //}
      //else
      //{
      //  std::cout << " " << m_dValues[j] << " ";
      //}

    }

  }
  myfile.close();
}

template<class T>
void MatrixCSR<T>::OutputMatrix()
{
  std::cout.precision(14);
	std::cout<<"------------------------------"<<std::endl;
	std::cout<<"Matrix output: "<<std::endl;

	for(int i=0;i<m_iN;i++)
	{
		for(int j=m_iRowPtr[i];j<m_iRowPtr[i+1];j++)
		{
			if(m_dValues[j]<0)
				std::cout<<m_dValues[j]<<" ";
			else
				std::cout<<" "<<m_dValues[j]<<" ";
		}
		std::cout<<std::endl;
	}
		std::cout<<"------------------------------"<<std::endl;
		std::cout<<std::endl;

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class MatrixCSR<float>;

template class MatrixCSR<double>;
//----------------------------------------------------------------------------

}
