#include <stdio.h>
#include <iostream>
#include <string.h>
#include "matrixnxn.h"

namespace i3d {

template<class T>
CMatrixNxN<T>::CMatrixNxN(void) : m_iN(0), m_iM(0) , m_v(NULL)
{
}

template<class T>
CMatrixNxN<T>::~CMatrixNxN(void)
{
	for(int i=0;i<m_iN;i++)
	{
		delete[] m_v[i];
	}
	delete[] m_v;
}

template<class T>
void CMatrixNxN<T>::OutputMatrix()
{
  std::cout.precision(14);
	std::cout<<"------------------------------"<<std::endl;
	std::cout<<"Matrix output: "<<std::endl;
	for(int i=0;i<m_iN;i++)
	{
		for(int j=0;j<m_iN;j++)
		{
			if(m_v[i][j]<0)
				std::cout<<m_v[i][j]<<" ";
			else
				std::cout<<" "<<m_v[i][j]<<" ";
		}
		std::cout<<std::endl;
	}
		std::cout<<"------------------------------"<<std::endl;
		std::cout<<std::endl;

}

template<class T>
int CMatrixNxN<T>::NumZeros() const
{
  int zeros = 0;
	for(int i=0;i<m_iN;i++)
	{
		for(int j=0;j<m_iN;j++)
		{
			if(m_v[i][j]==T(0.0))
      {
        zeros++;
      }
		}
	}
  return zeros;
}

template<class T>
CMatrixNxN<T>::CMatrixNxN(int n, int m, const T *pValues)
{
	m_v = new T*[n];
	m_iN=n;
	m_iM=m;
	for(int i=0;i<n;i++)
	{
		m_v[i] = new T[m];
	}

	for(int i=0;i<n;i++)
	{
		for(int j=0;j<n;j++)
		{
			m_v[i][j]=pValues[i*n+j];
		}
	}
	OutputMatrix();

}

	
template<class T>
CMatrixNxN<T>::CMatrixNxN(int n, int m)
{
	m_v = new T*[n];
	m_iN=n;
	m_iM=m;
	for(int i=0;i<n;i++)
	{
		m_v[i] = new T[m];
	}

	for(int i=0;i<n;i++)
	{
		for(int j=0;j<n;j++)
		{
			m_v[i][j]=(Real)0;
		}
	}
}


template<class T>
void CMatrixNxN<T>::SwapRows(int row0, int row1)
{
	T* pRow1 = new T[m_iM];
	memcpy(pRow1, m_v[row1], m_iM*sizeof(T));
	memcpy(m_v[row1],m_v[row0], m_iM*sizeof(T));
	memcpy(m_v[row0],pRow1, m_iM*sizeof(T));
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CMatrixNxN<float>;

template class CMatrixNxN<double>;
//----------------------------------------------------------------------------

}
