#if !defined(LINEARSOLVERLU_H)
#define LINEARSOLVERLU_H

//===================================================
//					INCLUDES
//===================================================

#include "matrixnxn.h"
#include "vectorn.h"

namespace i3d {

/**
* @brief A solver for a system of linear equations that uses LU-factorization
*
* A solver for a system of linear equations that uses LU-factorization
*/  
template<class T>
class CLinearSolverLU
{
public:
	CLinearSolverLU(void);
	~CLinearSolverLU(void);

	void Linsolve(CMatrixNxN<T> &A, CVectorN<T> &b, CVectorN<T> &x);

private:
	void SolveForwardBackward(CMatrixNxN<T>& A, CVectorN<T>& b, CVectorN<T>& x, int iperm[]);
};

}

#endif
