#include "linearsolverlu.h"
#include <iostream>

namespace i3d {

template<class T>
CLinearSolverLU<T>::CLinearSolverLU(void)
{
}

template<class T>
CLinearSolverLU<T>::~CLinearSolverLU(void)
{
}

template<class T>
void CLinearSolverLU<T>::Linsolve(MatrixNxN<T> &A, VectorN<T> &b, VectorN<T> &x)
{

	T sum=0;
	int i,j,k,n,p;

	n=A.rows();

	int *iperm = new int[n];


	for(i=0;i<n;i++)iperm[i]=i;

	for(j=0;j<=n-1;j++)
	{
		//partial pivoting
		p=j;
		//check for largest |a_ij| as pivot
		for(i=j+1;i<n;i++)
		{
			if(fabs(A(i,j)) > fabs(A(j,j)))
			{
				//swap rows if we find a bigger element
				A.SwapRows(j,i);
				T temp = b(j);
				b(j)=b(i);
				b(i)=temp;
			}
		}
		//check if there are only zero pivots
		while((A(p,j)==0.0) && (p<n))
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
			if(p !=j )
			{
				A.SwapRows(j,p);
				T temp = b(j);
				b(j)=b(p);
				b(p)=temp;
			}
		}


		//compute the betas : colums of U
		for(i=0;i<=j;i++)
		{
			sum=0.0;
			for(k=0;k<=i-1;k++)
				sum+=A(i,k)*A(k,j);

			//set the beta_ij
			A(i,j) = A(i,j) - sum;
		}//end i

		//compute the alphas colums of L
		for(i=j+1;i<=n-1;i++) 
		{
			sum=0.0;
			for(k=0;k<=j-1;k++)
			{
				//sum up the alphas before the diagonal and the
				//betas above the diagonal
				sum+=A(i,k)*A(k,j);
			}//end k

			//set alpha(i,j)
			A(i,j) = (1.0/A(j,j))*(A(i,j) - sum);
		}//end for i

	}//end for j

	SolveForwardBackward(A,b,x,iperm);
}


template<typename T>
void CLinearSolverLU<T>::SolveForwardBackward(MatrixNxN<T>& A, VectorN<T>& b, VectorN<T>& x, int iperm[])
{

	int i,j,k,n;

	n=A.rows();

	for(i=0;i<n;i++)
	{
		j = iperm[i];
		x(i)=b(j);
	}

	x(0) = x(0);

	for(i=1;i<n;i++)
	{
		T sum=0.0;
		for(k=0;k<i;k++)
			sum+=A(i,k)*x(k);
		x(i)=(x(i)-sum);
	}

	//x(n-1) is already done, because of:
	x(n-1)=x(n-1)/A(n-1,n-1);

	for(i=n-2;i>=0;i--)
	{
		T sum = 0.0;
		for(k=i+1;k<n;k++)
			sum += A(i,k)*x(k);
		x(i)=(x(i)-sum)/A(i,i);
	}

	x.OutputVector();

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CLinearSolverLU<float>;

template class CLinearSolverLU<double>;
//----------------------------------------------------------------------------

}