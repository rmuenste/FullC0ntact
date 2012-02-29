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

#ifndef LEMKESOLVER_H
#define LEMKESOLVER_H

namespace i3d {

/**
* @brief A LCP solver using Lemke's algorithm
*
* A direct LCP solver that implements the Lemke-Howson algorithm
* to solve the LCP w = Mz + q, w o z = 0, w >= 0, z >= 0.
*/
class CLemkeSolver
{
public:

  CLemkeSolver();

/**
* Starts the LCP-solver
*
* @param numEquations Number of variables
* @param M the M matrix of the problem
* @param Q the q vector of the problem
* @param Z the z vector of the problem
* @param W the w vector of the problem
* @param status the return status of the solver
* @param maxRetries the maximum number of restarts
* @param zeroTolerance the zero tolerance used in the solver
* @param ratioError the ratio error
*
*/
  CLemkeSolver (int numEquations, double** M, double* Q, double* Z,
      double* W, int& status, int maxRetries = 100,
      double zeroTolerance = 0.0, double ratioError = 0.0);

  enum // status codes
  {
      SC_FOUND_SOLUTION,               // solution
      SC_FOUND_TRIVIAL_SOLUTION,       // solution (z = 0, w = q)
      SC_CANNOT_REMOVE_COMPLEMENTARY,  // no solution (unbounded)
      SC_EXCEEDED_MAX_RETRIES,         // no solution (round-off problems?)
  };

  // MaxRetries:  In theory, one iteration of the LCP solver should work.
  // Floating point round-off errors can cause the solver to fail.  When
  // this does, the solver is allowed to retry the algorithm with a
  // different order of input equations.

  // ZeroTolerance:  The search for a pivot equation uses a comparison of a
  // certain term to zero.  To deal with floating point round-off errors,
  // the comparison is based on a small tolerance about zero.

  // RatioError:  The solver computes constant coefficients, z coefficients,
  // and w coefficients.  If any of these are nearly zero, their values are
  // set to zero.  The decision is made based on a relative comparison of a
  // ratio to zero.

private:
  void AllocateEquations ();
  void DeallocateEquations ();
  bool InitializeEquations ();
  bool SelectEquation (int& equation);
  bool FindEquation (int& equation);
  bool EquationAlgorithm (int& equation);
  void Solve (char basicVariable, int basicVariableIndex);

  struct Equation
  {
      char Var;      // 'w' or 'z' are the only choices
      int VarIndex;  // index of the w or z variable
      double* C;     // constant coefficients
      double* W;     // coefficients of the w terms
      double* Z;     // coefficients of the z terms
  };

  int mNumEquations;
  double** mM;
  double* mQ;
  Equation* mEquations;
  char mNonBasicVariable;
  char mDepartingVariable;
  int mNonBasicVariableIndex;
  int mDepartingVariableIndex;
  double mFuzz;

  int mMaxRetries;
  double mZeroTolerance;
  double mRatioError;

};

}

#endif
