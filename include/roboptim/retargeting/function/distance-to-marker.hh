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
# include <roboptim/core/numeric-quadratic-function.hh>
# include <roboptim/core/filter/chain.hh>

# include <roboptim/retargeting/function/joint-to-marker/choreonoid.hh>

namespace roboptim
{
  namespace retargeting
  {
    namespace detail
    {
      template <typename T>
      class DistanceToReference : public GenericNumericQuadraticFunction<T>
      {
      public:
	ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
	(GenericNumericQuadraticFunction<T>);

	explicit DistanceToReference (const vector_t& reference)
	  : GenericNumericQuadraticFunction<T>
	    (matrix_t (reference.size (), reference.size ()),
	     vector_t (reference.size ()),
	     vector_t (1))
	{
	  this->A ().setIdentity ();
	  this->b () = -2. * reference;
	  this->c ()[0] = reference.squaredNorm ();
	}
      };

      template <typename T>
      boost::shared_ptr<GenericDifferentiableFunction<T> >
      distanceToMarkerInternal
      (const typename GenericDifferentiableFunction<T>::vector_t& references,
       boost::shared_ptr<GenericDifferentiableFunction<T> > jointToMarker)
      {
	boost::shared_ptr<GenericDifferentiableFunction<T> >
	  distanceToRef =
	  boost::make_shared<DistanceToReference<T> > (references);

	boost::shared_ptr<GenericDifferentiableFunction<T> >
	  f = chain (distanceToRef, jointToMarker);
	return f;
      }
    } // end of namespace detail



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
	  f_ (detail::distanceToMarkerInternal<T>
	      (markersReferencePosition, jointToMarker)),
	  nMarkers_
	  (static_cast<typename vector_t::Index>
	   (jointToMarker->outputSize () / 3))
      {}

      virtual ~DistanceToMarker ()
      {}

      void
      impl_compute
      (result_t& result, const argument_t& x)
	const
      {
	result = (*f_)(x);
	result /= 2. * nMarkers_;
      }

      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type i)
	const
      {
	f_->gradient (gradient, x, i);
	gradient /= 2. * nMarkers_;
      }

    private:
      boost::shared_ptr<GenericDifferentiableFunction<T> > f_;
      typename vector_t::Index nMarkers_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_DISTANCE_TO_MARKER_HH
