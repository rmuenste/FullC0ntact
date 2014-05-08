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

	void Linsolve(MatrixNxN<T> &A, VectorN<T> &b, VectorN<T> &x);

private:
	void SolveForwardBackward(MatrixNxN<T>& A, VectorN<T>& b, VectorN<T>& x, int iperm[]);
};

}

#endif
