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
#ifndef ROBOPTIM_RETARGETING_PROBLEM_PROBLEM_BUILDER_HH
# define ROBOPTIM_RETARGETING_PROBLEM_PROBLEM_BUILDER_HH
# include <boost/shared_ptr.hpp>

# include <roboptim/core/differentiable-function.hh>
# include <roboptim/core/linear-function.hh>
# include <roboptim/core/problem.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Define dense problem type.
    typedef roboptim::Problem<
      roboptim::GenericDifferentiableFunction<EigenMatrixDense>,
      boost::mpl::vector<
	roboptim::GenericLinearFunction<EigenMatrixDense>,
	roboptim::GenericDifferentiableFunction<EigenMatrixDense>
	>
    >
    denseProblem_t;

    /// \brief Define sparse problem type.
    typedef roboptim::Problem<
      roboptim::GenericDifferentiableFunction<EigenMatrixSparse>,
      boost::mpl::vector<
	roboptim::GenericLinearFunction<EigenMatrixSparse>,
	roboptim::GenericDifferentiableFunction<EigenMatrixSparse>
	>
      >
    sparseProblem_t;

    /// \brief Abstract Base Class for problem builders.
    ///
    /// A problem builder is a class builder a RobOptim problem.
    ///
    /// It takes as input strings describing which cost function to
    /// use, which constraints should be enabled, what trajectory
    /// representation should be chosen, etc.
    ///
    /// Once the problem is built, it can be retrieved using the
    /// operator ().
    ///
    /// \tparam T problem type
    template <typename T>
    class ProblemBuilder
    {
    public:
      ProblemBuilder ()
      {}
      virtual ~ProblemBuilder ()
      {}

      virtual std::pair<boost::shared_ptr<T>,
			boost::shared_ptr<typename T::function_t> >
      operator () () = 0;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_PROBLEM_PROBLEM_BUILDER_HH
