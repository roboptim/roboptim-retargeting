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
    ()
      : Trajectory<3> (makeInterval (0., 1.), 1,
		       vector_t::Zero (4),
		       "minimum jerk trajectory"),
	coefficients_ ()
    {}

    template <typename T>
    MinimumJerkTrajectory<T>::~MinimumJerkTrajectory ()
    {}

    template <typename T>
    void
    MinimumJerkTrajectory<T>::setParameters (const vector_t& params)
    {
      Trajectory<3>::setParameters (params);

      value_type positionStart = params[0];
      value_type positionEnd = params[1];
      value_type velocityStart = params[2];
      value_type accelerationStart = params[3];

      value_type length = this->length ();
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
    void
    MinimumJerkTrajectory<T>::impl_compute (result_t& result, double t)
      const
    {
      if (t < timeRange ().first)
	{
	  result[0] = parameters ()[0];
	  return;
	}
      if (t > timeRange ().second)
	{
	  result[0] = parameters ()[1];
	  return;
	}

      // accumulate the power of t (t)
      value_type accu = 1.;
      value_type tScaled = (t - timeRange ().first) / this->length ();

      result[0] = 0.;
      for (std::size_t i = 0; i < coefficients_.size (); ++i)
	{
	  result[0] += coefficients_[i] * accu;
	  accu *= tScaled;
	}
    }

    template <typename T>
    typename MinimumJerkTrajectory<T>::jacobian_t
    MinimumJerkTrajectory<T>::variationConfigWrtParam (double) const
    {
      throw std::runtime_error ("NOT IMPLEMENTED");
    }

    template <typename T>
    typename MinimumJerkTrajectory<T>::jacobian_t
    MinimumJerkTrajectory<T>::variationDerivWrtParam (double, size_type)
      const
    {
      throw std::runtime_error ("NOT IMPLEMENTED");
    }

    template <typename T>
    typename MinimumJerkTrajectory<T>::value_type
    MinimumJerkTrajectory<T>::singularPointAtRank (size_type) const
    {
      return value_type (); // zero
    }

    template <typename T>
    typename MinimumJerkTrajectory<T>::vector_t
    MinimumJerkTrajectory<T>::derivBeforeSingularPoint (size_type, size_type) const
    {
      throw std::runtime_error ("NOT IMPLEMENTED");
    }

    template <typename T>
    typename MinimumJerkTrajectory<T>::vector_t
    MinimumJerkTrajectory<T>::derivAfterSingularPoint (size_type, size_type) const
    {
      throw std::runtime_error ("NOT IMPLEMENTED");
    }

    template <typename T>
    typename MinimumJerkTrajectory<T>::jacobian_t
    MinimumJerkTrajectory<T>::variationConfigWrtParam (StableTimePoint) const
    {
      throw std::runtime_error ("NOT IMPLEMENTED");
    }

    template <typename T>
    typename MinimumJerkTrajectory<T>::jacobian_t
    MinimumJerkTrajectory<T>::variationDerivWrtParam
    (StableTimePoint, size_type) const
    {
      throw std::runtime_error ("NOT IMPLEMENTED");
    }

    template <typename T>
    void
    MinimumJerkTrajectory<T>::impl_derivative (gradient_t& gradient,
					       double t,
					       size_type order)
      const
    {
      gradient.setZero ();
      if (t < timeRange ().first || t > timeRange ().second)
	return;

      if (order == 0)
	this->operator () (gradient, t);
      else if (order == 1)
	{
	  // accumulate the power of t
	  value_type accu = 1.;
	  value_type tScaled = (t - timeRange ().first) / this->length ();
	  value_type i = 1.;

	  for (std::size_t idx = 1; idx < coefficients_.size (); ++idx, i += 1.)
	    {
	      gradient[0] += i * coefficients_[idx] * accu;
	      accu *= tScaled;
	    }
	}
      else if (order == 2)
	{
	  // accumulate the power of t
	  value_type accu = 1.;
	  value_type tScaled = (t - timeRange ().first) / this->length ();
	  value_type i = 2.;

	  for (std::size_t idx = 2; idx < coefficients_.size (); ++idx, i += 1.)
	    {
	      gradient[0] += i * (i - 1.) * coefficients_[idx] * accu;
	      accu *= tScaled;
	    }
	}
    }

    template <typename T>
    void
    MinimumJerkTrajectory<T>::impl_derivative (gradient_t&, StableTimePoint, size_type)
      const
    {
      throw std::runtime_error ("NOT IMPLEMENTED");
    }

    template <typename T>
    Trajectory<3>*
    MinimumJerkTrajectory<T>::resize (interval_t)
      const
    {
      throw std::runtime_error ("NOT IMPLEMENTED");
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_MINIMUM_JERK_TRAJECTORY_HXX
