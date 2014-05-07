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

#ifndef ROBOPTIM_RETARGETING_FUNCTION_FACTORY_HH
# define  ROBOPTIM_RETARGETING_FUNCTION_FACTORY_HH
# include <vector>
# include <boost/shared_ptr.hpp>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Constraint definition
    ///
    /// A constraint is defined as a triplet containing:
    /// - a (shared pointer to a) function
    /// - an interval over which the constraint is satisfied
    /// - a scale factor used to evaluate the function in order to
    ///   obtain a better numerical stability
    template <typename T>
    struct Constraint
    {
      /// \brief Define how the constraint will be added to the problem.
      ///
      /// Two possibilities here: either the constraint is applied on
      /// the whole trajectory or on each trajectory state. In this
      /// case, the state function may depend on dq or ddq. This is
      /// indicated by the "state function order" attribute.
      enum ConstraintType
	{
	  /// \brief This constraint will be applied once.
	  ///
	  /// This constraint function input size must be equal to the
	  /// whole filtered trajectory parameters vector size.
	  CONSTRAINT_TYPE_ONCE = 0,

	  /// \brief This constraint must be repeated for each frame
	  /// excluding the first and last one.
	  ///
	  /// This constraint function input size must be equal to the
	  /// whole filtered trajectory parameters vector size.
	  CONSTRAINT_TYPE_PER_FRAME
	};

      /// \brief Shared pointer to function
      boost::shared_ptr<T> function;
      /// \brief Interval on which the constraint is valid
      typename T::intervals_t intervals;
      /// \brief Scaling to be used for this constraint
      std::vector<typename T::value_type> scales;

      /// \brief One-time constraint or per-frame constraint?
      ///
      /// If the constraint is a "one time" constraint then it will be
      /// added in the problem as it is. If this is a per-frame
      /// constraint, it will be added for each frame of the problem
      /// (excluding the first and last one).
      ConstraintType type;

      /// \brief Which derivative order is required by this constraint?
      ///
      /// 0 means the state function depends on q (i.e. the current
      /// configuration as produced by trajectory evaluation at a
      /// particular time).
      /// 1 means q and dq
      /// 2 means q, dq, ddq
      /// etc.
      ///
      /// See roboptim-trajectory StateFunction for more information.
      ///
      /// \note This attribute is ignored if the constraint type is
      ///       set to CONSTRAINT_TYPE_ONCE.
      std::size_t stateFunctionOrder;
    };

    /// \brief Function factories inherit from this class (empty for
    ///        now).
    class FunctionFactory
    {};
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_FACTORY_HH
