#include <iostream>
#include "linearsolvergauss.h"

namespace i3d {

template<class T>
CLinearSolverGauss<T>::CLinearSolverGauss(void)
{

}

template<class T>
CLinearSolverGauss<T>::~CLinearSolverGauss(void)
{

}

template<class T>
void CLinearSolverGauss<T>::Linsolve(MatrixNxN<T> &A, VectorN<T> &b, VectorN<T> &x)
{
	int n = A.m_iN;
	int k=0;

	int p;
	for(;k<n-1;k++)
	{
		p=k;

		//check for largest |a_ij| as pivot
		for(int i=k+1;i<n;i++)
		{
			if(fabs(A(i,k)) > fabs(A(k,k)))
			{
				//swap rows if we find a bigger element
				A.SwapRows(k,i);
				T temp = b(k);
				b(k)=b(i);
				b(i)=temp;
			}
		}
		//check if there are only zero pivots
		while((A(p,k)==0.0) && (p<n))
		{
			p++;
		}

		if(p==n)
		{
			std::cout<<"No solution exists..."<<std::endl;
			return;
		}
		else
		{
			if(p !=k )
			{
				A.SwapRows(k,p);
				T temp = b(k);
				b(k)=b(p);
				b(p)=temp;
			}
		}
		
		for(int i=k+1;i<n;i++)
		{
			T p = A(i,k);
			T pivot = A(i,k)/A(k,k);

			//reduce the k+1-row
			for(int j=0;j<n;j++)
				A(i,j)=A(i,j) - pivot * A(k,j);

			//reduce the rhs
			b(i) = b(i) - pivot * b(k);
		}//end for i
	}//end for k

	if(A(n-1,n-1) == 0)
	{
		std::cout<<"No solution exists..."<<std::endl;
		return;
	}

	//backwards substitution
	for(int i=n-1;i>=0;i--)
	{
		T Sum = 0;
		x(i)=0;
		for(int j=i; j<=n-1;j++)
		{
			Sum += A(i,j) * x(j);
		}
		x(i)=(b(i)-Sum)/A(i,i);
	}

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CLinearSolverGauss<float>;

template class CLinearSolverGauss<double>;
//----------------------------------------------------------------------------

}