#if !defined(_CMATRIXNXN_H)
#define _CMATRIXNXN_H

//===================================================
//					INCLUDES
//===================================================
#include "mymath.h"
#include <mathglobals.h>

namespace i3d {

/**
* @brief A nxn matrix
*
* A nxn matrix, intended for use with n > 4
*/  
template<class T>
class CMatrixNxN
{
public:
	CMatrixNxN(void);
	~CMatrixNxN(void);

	CMatrixNxN(int n, int m, const T *pValues);
	CMatrixNxN(int n, int m);

	inline T& operator() (unsigned int row, unsigned int col)
	{
		return m_v[row][col];
	}//end operator()

	inline T operator() (unsigned int row, unsigned int col) const
	{
		return m_v[row][col];
	}//end operator()  

	inline int rows() const
	{
		return m_iN;
	}//end rows()

	void SwapRows(int row0, int row1);

	void OutputMatrix();

    void Free();

  int NumZeros() const;

	int m_iN;
	int m_iM;

	T **m_v;

};


typedef CMatrixNxN<double> CMatrixNxNd;
typedef CMatrixNxN<float>  CMatrixNxNf;
typedef CMatrixNxN<Real>   CMatrixNxNr;

}

#endif
