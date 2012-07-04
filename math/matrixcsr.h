#if !defined(_CMATRIXCSR_H)
#define _CMATRIXCSR_H

//===================================================
//					INCLUDES
//===================================================
#include "mymath.h"
#include <mathglobals.h>
#include <matrixnxn.h>

namespace i3d {

/**
* @brief A nxn matrix
*
* A nxn matrix, intended for use with n > 4
*/  
template<class T>
class CMatrixCSR
{
public:
	CMatrixCSR(void);

  CMatrixCSR(const CMatrixNxN<T> &matrix);

  CMatrixCSR(int n, int entries);

  CMatrixCSR(int n, int entries, int *rowPointer);

	~CMatrixCSR(void);

  void OutputMatrix();

	int m_iNumVal;

  int m_iN;

	T   *m_dValues;
  int *m_iColInd;
  int *m_iRowPtr;

};

typedef CMatrixCSR<double> CMatrixCSRd;
typedef CMatrixCSR<float>  CMatrixCSRf;
typedef CMatrixCSR<Real>   CMatrixCSRr;

}

#endif
