#if !defined(LINEARSOLVERGAUSS_H)
#define LINEARSOLVERGAUSS_H

//===================================================
//					INCLUDES
//===================================================

#include "matrixnxn.h"
#include "vectorn.h"

namespace i3d {

/**
* @brief A gauss solver for a system of linear equations
*
* A gauss solver for a system of linear equations
*/  
template<class T>
class CLinearSolverGauss
{
public:
	CLinearSolverGauss(void);
	~CLinearSolverGauss(void);

	void Linsolve(CMatrixNxN<T> &A, CVectorN<T> &b, CVectorN<T> &x);

};

}

#endif
