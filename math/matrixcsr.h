#if !defined(_CMATRIXCSR_H)
#define _CMATRIXCSR_H

//===================================================
//					INCLUDES
//===================================================
#include "mymath.h"
#include <string>
#include <mathglobals.h>
#include <matrixnxn.h>

namespace i3d {

/**
* @brief A nxn matrix
*
* A nxn matrix, intended for use with n > 4
*/  
template<class T>
class MatrixCSR
{
public:

  MatrixCSR(void);

  MatrixCSR(const MatrixNxN<T> &matrix);

  MatrixCSR(int n, int entries, int *rowPointer);

	~MatrixCSR(void);

  void OutputMatrix();

  void outputToFile(std::string fileName);

	int m_iNumVal;

  int m_iN;

	T   *m_dValues;
  int *m_iColInd;
  int *m_iRowPtr;

};

typedef MatrixCSR<double> MatrixCSRd;
typedef MatrixCSR<float>  MatrixCSRf;
typedef MatrixCSR<Real>   MatrixCSRr;

}

#endif
