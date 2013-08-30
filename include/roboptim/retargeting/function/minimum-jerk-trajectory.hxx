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

#ifndef ROBOPTIM_RETARGETING_MINIMUM_JERK_TRAJECTORY_HXX
# define ROBOPTIM_RETARGETING_MINIMUM_JERK_TRAJECTORY_HXX
# include <roboptim/retargeting/function/minimum-jerk-trajectory.hh>

namespace roboptim
{
  namespace retargeting
  {
    template <typename T>
    MinimumJerkTrajectory<T>::MinimumJerkTrajectory
    (value_type timeStart, value_type timeEnd,
     value_type positionStart, value_type positionEnd,
     value_type velocityStart, value_type accelerationStart) throw ()
      : GenericTwiceDifferentiableFunction<T> (1, 1, "minimum jerk trajectory"),
	timeStart_ (timeStart),
	timeEnd_ (timeEnd),
	positionStart_ (positionStart),
	positionEnd_ (positionEnd),
	coefficients_ ()
    {
      assert (timeEnd >= timeStart);

      value_type length = timeEnd - timeStart;
      value_type delta = positionEnd - positionStart;

      coefficients_[0] = positionStart;
      coefficients_[1] = velocityStart;
      coefficients_[2] = .5 * accelerationStart;
      coefficients_[3] =
	- 3. / 2. * length * length * accelerationStart
	- 6. * length * velocityStart
	+ 10 * delta;
      coefficients_[4] =
	3. / 2. * length * length * accelerationStart
	+ 8 * length * velocityStart
	- 15 * delta;
      coefficients_[5] =
	- 1. / 2. * length * length * accelerationStart
	- 3. * length * velocityStart
	+ 6 * delta;
    }

    template <typename T>
    MinimumJerkTrajectory<T>::~MinimumJerkTrajectory () throw ()
    {}

    template <typename T>
    void
    MinimumJerkTrajectory<T>::impl_compute (result_t& result,
					    const argument_t& argument)
      const throw ()
    {
      if (argument[0] < timeStart_)
	{
	  result[0] = positionStart_;
	  return;
	}
      if (argument[0] > timeEnd_)
	{
	  result[0] = positionEnd_;
	  return;
	}

      // accumulate the power of t (argument[0])
      value_type accu = 1.;
      value_type t = (argument[0] - timeStart_) / (timeEnd_ - timeStart_);

      result[0] = 0.;
      for (std::size_t i = 0; i < coefficients_.size (); ++i)
	{
	  result[0] += coefficients_[i] * accu;
	  accu *= t;
	}
    }

    template <typename T>
    void
    MinimumJerkTrajectory<T>::impl_gradient (gradient_t& gradient,
					     const argument_t& argument,
					     size_type)
      const throw ()
    {
      if (argument[0] < timeStart_ || argument[0] > timeEnd_)
	{
	  gradient.setZero ();
	  return;
	}

      // accumulate the power of t
      value_type accu = 1.;
      value_type t = (argument[0] - timeStart_) / (timeEnd_ - timeStart_);
      value_type i = 1.;

      gradient[0] = 0.;
      for (std::size_t idx = 1; idx < coefficients_.size (); ++idx, i += 1.)
	{
	  gradient[0] += i * coefficients_[idx] * accu;
	  accu *= t;
	}
    }

    template <typename T>
    void
    MinimumJerkTrajectory<T>::impl_hessian (hessian_t& hessian,
					    const argument_t& argument,
					    size_type)
      const throw ()
    {
      if (argument[0] < timeStart_ || argument[0] > timeEnd_)
	{
	  hessian.setZero ();
	  return;
	}

      // accumulate the power of t
      value_type accu = 1.;
      value_type t = (argument[0] - timeStart_) / (timeEnd_ - timeStart_);
      value_type i = 2.;

      hessian (0, 0) = 0.;
      for (std::size_t idx = 2; idx < coefficients_.size (); ++idx, i += 1.)
	{
	  hessian (0, 0) += i * (i - 1.) * coefficients_[idx] * accu;
	  accu *= t;
	}
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_MINIMUM_JERK_TRAJECTORY_HXX
