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
# include <roboptim/trajectory/trajectory.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Minimum jerk trajectory computation.
    ///
    /// The curve is parametrized by a vector of four parameters:
    /// - position start
    /// - position end
    /// - velocity start
    /// - acceleration start
    ///
    /// This class is a partial implementation of a Trajectory.
    /// Some methods will return a runtime_error if called.
    template <typename T>
    class MinimumJerkTrajectory :
      public Trajectory<3>
    {
    public:
      ROBOPTIM_TWICE_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericTwiceDifferentiableFunction<T>);

      ROBOPTIM_IMPLEMENT_CLONE (MinimumJerkTrajectory<T>);

      explicit MinimumJerkTrajectory () throw ();
      virtual ~MinimumJerkTrajectory () throw ();

      /// \brief Store parameters and update coefficients.
      void setParameters (const vector_t&) throw ();
    protected:
      void impl_compute (result_t& result, double t) const throw ();
      jacobian_t variationConfigWrtParam (double t) const throw ();
      jacobian_t variationDerivWrtParam (double t, size_type order)
	const throw ();
      value_type singularPointAtRank (size_type rank) const;
      vector_t derivBeforeSingularPoint (size_type rank, size_type order) const;
      vector_t derivAfterSingularPoint (size_type rank, size_type order) const;

      jacobian_t variationConfigWrtParam (StableTimePoint tp)
      const throw ();
      jacobian_t
      variationDerivWrtParam (StableTimePoint tp, size_type order)
      const throw ();
      void impl_derivative (gradient_t& derivative,
			    double argument,
			    size_type order = 1) const throw ();
      void impl_derivative (gradient_t& g, StableTimePoint, size_type order)
	const throw ();
      Trajectory<3>* resize (interval_t timeRange)
	const throw ();
    private:
      boost::array<value_type, 6> coefficients_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

# include <roboptim/retargeting/function/minimum-jerk-trajectory.hxx>
#endif //! ROBOPTIM_RETARGETING_MINIMUM_JERK_TRAJECTORY_HH
