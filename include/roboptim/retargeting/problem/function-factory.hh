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
      /// \brief Shared pointer to function
      boost::shared_ptr<T> function;
      /// \brief Interval on which the constraint is valid
      typename T::intervals_t intervals;
      /// \brief Scaling to be used for this constraint
      std::vector<typename T::value_type> scales;
    };

    /// \brief Function factories inherit from this class (empty for
    ///        now).
    class FunctionFactory
    {};
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_FACTORY_HH
