// Copyright (C) 2013 by Thomas Moulard, AIST, CNRS, INRIA.
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

#ifndef ROBOPTIM_RETARGETING_MINIMUM_JERK_TRAJECTORY_HH
# define ROBOPTIM_RETARGETING_MINIMUM_JERK_TRAJECTORY_HH
# include <boost/array.hpp>
# include <boost/shared_ptr.hpp>
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Minimum jerk trajectory computation.
    template <typename T>
    class MinimumJerkTrajectory :
      public GenericDifferentiableFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_ (GenericDifferentiableFunction<T>);

      explicit MinimumJerkTrajectory
      (value_type timeStart, value_type timeEnd,
       value_type positionStart, value_type positionEnd,
       value_type velocityStart = 0.,
       value_type accelerationStart = 0.) throw ();
      virtual ~MinimumJerkTrajectory () throw ();

    protected:
      virtual void impl_compute (result_t& result,
				 const argument_t& argument)
	const throw ();


      virtual void impl_gradient (gradient_t& gradient,
				  const argument_t& argument,
				  size_type functionId = 0)
	const throw ();

    private:
      value_type timeStart_;
      value_type timeEnd_;
      value_type positionStart_;
      value_type positionEnd_;
      boost::array<value_type, 6> coefficients_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

# include <roboptim/retargeting/function/minimum-jerk-trajectory.hxx>
#endif //! ROBOPTIM_RETARGETING_MINIMUM_JERK_TRAJECTORY_HH
