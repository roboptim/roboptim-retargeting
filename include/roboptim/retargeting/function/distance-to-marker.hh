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
      /// \brief Quadratic function computing the distance between a
      ///        vector and a reference.
      ///
      /// This formula is widely used as a cost function. In this
      /// particular case, it will be used by DistanceToMarker class
      /// (by chaining JointToMarker and this class).
      ///
      /// Input: x (size: reference vector size)
      /// Output: \f$ \sum (x_i - r_i)^2 \f$ (size: 1)
      ///
      /// This should be probably promoted as a generic class in
      /// RobOptim Core at some point.
      ///
      /// \tparam T function trait
      template <typename T>
      class DistanceToReference : public GenericNumericQuadraticFunction<T>
      {
      public:
	ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
	(GenericNumericQuadraticFunction<T>);

	/// \brief Constructor
	///
	/// Build the quadratic function from the reference vector.
	///
	/// \param[in] reference vector
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
    /// This function takes a robot configuration as input. Using
    /// JointToMarker function, it computes where the markers would be
    /// in this particular configuration. Then it computes the
    /// distance between the current position and the expected,
    /// constant, ones. The reference positions are determined when
    /// this function is built.
    ///
    /// Input: robot configuration (size: 6 + number of DOFs)
    /// Output: cost (size: 1)
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


      /// \brief Constructor
      ///
      /// \param[in] jointToMarker Shared pointer to a JointToMarker
      ///                      function (necessary to compute the
      ///                      relative position of markers w.r.t
      ///                      robot bodies)
      ///
      /// \param[in] markersReferencePosition Expected markers
      ///                      positions (size: 3 * number of markers)
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

    protected:
      void
      impl_compute
      (result_t& result, const argument_t& x)
	const
      {
	result = (*f_)(x);
	result /= 2. * static_cast<value_type> (nMarkers_);
      }

      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type i)
	const
      {
	f_->gradient (gradient, x, i);
	gradient /= 2. * static_cast<value_type> (nMarkers_);
      }

    private:
      boost::shared_ptr<GenericDifferentiableFunction<T> > f_;
      typename vector_t::Index nMarkers_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_DISTANCE_TO_MARKER_HH
