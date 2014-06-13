// Copyright (C) 2014 by Thomas Moulard, AIST, CNRS.
//
// This file is part of the roboptim.
//
// roboptim is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// roboptim is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with roboptim.  If not, see <http://www.gnu.org/licenses/>.

#ifndef ROBOPTIM_RETARGETING_FUNCTION_RSS_HH
# define ROBOPTIM_RETARGETING_FUNCTION_RSS_HH
# include <roboptim/core/numeric-quadratic-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Residual Sum of Squares (RSS)
    ///
    /// Input:
    /// x = [x0, x1, ..., xN] (size: N)
    ///
    /// Output:
    /// f(x) >= 0 (size: 1)
    ///
    /// This functions takes as input a reference vector and computes
    /// the Residual Sum of Squares.
    ///
    /// It returns the squared norm of the difference between this
    /// original value and the current input (i.e. the current
    /// Laplacian Coordinates) and is expressed as a quadratic
    /// function.
    ///
    /// f(x) = || S - X ||^2 = (S - X)^T . (S - X)
    ///      = S^2 - 2 . S . X + X^2
    ///
    /// A = I
    /// B = -2 * S
    /// C = S^T * S
    ///
    /// f(x) = X^T * I * X + B * X + C
    ///      = X^2 - 2 * S * X + S^2
    ///
    /// \param reference reference values, this function will reach
    ///        its minimum value at this point.
    ///
    /// \tparam T Function traits type
    template <typename T>
    class RSS : public GenericNumericQuadraticFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericDifferentiableFunction<T>);

      explicit RSS (const vector_t& ref)
	: GenericNumericQuadraticFunction<T>
	  (matrix_t (ref.size (), ref.size ()),
	   vector_t (ref.size ()))
      {
	this->A ().setIdentity ();

	this->b () = ref;
	this->b () *= -2.;

	this->c () = ref.adjoint () * ref;
      }
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_RSS_HH
