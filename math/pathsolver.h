/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#ifndef PATHSOLVER_H
#define PATHSOLVER_H

namespace i3d {

/**
* @brief An interface to Michael C. Ferris' PATH solver
*
* An interface to Michael C. Ferris' PATH solver
*/
class CPathSolver
{
public:
		// A class for solving the Linear Complementarity Problem (LCP)
		// w = Mz + q, w o z = 0, w >= 0, z >= 0.
		//
		// Input:
		//   'numEquations' is the number of equations.
		//   'M' is a positive semidefinite matrix of size 'numEquations'.
		//   'Q' is a vector of reals.
		//

		CPathSolver();
		CPathSolver (int numEquations, double** M, double* Q, double* Z,
				double* W, int& status, int maxRetries = 100,
				double zeroTolerance = 0.0, double ratioError = 0.0);

		~CPathSolver();

  void Solve();

private:

  void ComputeNonZeroEntries();
  void GeneratePathFormat();
  
  int m_nnz;
  int *m_i;
  int *m_j;


  int mNumEquations;


  double *m_ij;
  double *m_lb;
  double *m_ub;
  double *m_q;
  double *m_z;

  double **mM;
  double *mQ;
  double *mZ;

  double m_ZeroTolerance;
  int mMaxRetries;
  double mZeroTolerance;
  double mRatioError;

};

}

#endif
