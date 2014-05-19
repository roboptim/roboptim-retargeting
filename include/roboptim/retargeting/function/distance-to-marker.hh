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

#ifndef ROBOPTIM_RETARGETING_FUNCTION_DISTANCE_TO_MARKER_HH
# define ROBOPTIM_RETARGETING_FUNCTION_DISTANCE_TO_MARKER_HH
# include <string>

# include <roboptim/core/differentiable-function.hh>
# include <roboptim/core/finite-difference-gradient.hh>

# include <roboptim/retargeting/function/joint-to-marker/choreonoid.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Distance between markers expected positions and
    ///        reference markers positions.
    ///
    /// This is used as cost function for the marker to joints
    /// problem.
    ///
    /// FIXME: document
    ///
    /// \tparam T Function traits type
    template <typename T>
    class DistanceToMarker : public GenericDifferentiableFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericDifferentiableFunction<T>);

      typedef boost::shared_ptr<JointToMarkerPositionChoreonoid<T> >
      JointToMarkerShPtr_t;


      explicit DistanceToMarker
      (JointToMarkerShPtr_t jointToMarker,
       const vector_t& markersReferencePosition)
	: GenericDifferentiableFunction<T>
	  (jointToMarker->inputSize (), 1, "DistanceToMarker"),
	  jointToMarker_ (jointToMarker),
	  markersReferencePosition_ (markersReferencePosition),
	  fd_ (boost::make_shared<fdFunction_t> (*this))
      {}

      virtual ~DistanceToMarker ()
      {}

      void
      impl_compute
      (result_t& result, const argument_t& x)
	const
      {
	vector_t markersPositions = (*jointToMarker_) (x);

	result[0] = 0.;

	typename vector_t::Index nMarkers =
	  static_cast<typename vector_t::Index> (jointToMarker_->outputSize () / 3);
	for (typename vector_t::Index markerId = 0; markerId < nMarkers; ++markerId)
	  {
	    vector_t delta =
	      markersReferencePosition_.template segment<3> (markerId * 3)
	      - markersPositions.template segment<3> (markerId * 3);
	    result[0] += delta.squaredNorm ();
	  }

	result[0] *= 1. / 2.;

	// normalize result
	result[0] /= nMarkers;
      }

      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type i)
	const
      {
	fd_->gradient (gradient, x, i);
      }

    private:
      JointToMarkerShPtr_t jointToMarker_;
      vector_t markersReferencePosition_;

      // Temporary - finite difference gradient.
      // Currently the simple policy generates precision errors,
      // one may want to switch to five point rules.
      typedef roboptim::GenericFiniteDifferenceGradient<
	T, finiteDifferenceGradientPolicies::Simple<T> >
      fdFunction_t;
      boost::shared_ptr<fdFunction_t> fd_;

    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_DISTANCE_TO_MARKER_HH
