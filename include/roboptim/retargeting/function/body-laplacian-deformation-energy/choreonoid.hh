#ifndef ROBOPTIM_RETARGETING_FUNCTION_BODY_LAPLACIAN_DEFORMATION_ENERGY_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FUNCTION_BODY_LAPLACIAN_DEFORMATION_ENERGY_CHOREONOID_HH
# include <stdexcept>

# include <boost/make_shared.hpp>

# include <roboptim/core/differentiable-function.hh>

# include <roboptim/retargeting/function/joint-to-marker/choreonoid.hh>

# include <cnoid/BodyIMesh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Laplacian Deformation Energy computed using Choreonoid
    ///        Motion Capture plug-in.
    ///
    /// Input:
    ///  x = [q]
    ///
    /// Output:
    ///  result = [cost]
    ///
    /// \tparam T Function traits type
    template <typename T>
    class BodyLaplacianDeformationEnergyChoreonoid
      : public GenericDifferentiableFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericDifferentiableFunction<T>);

      explicit BodyLaplacianDeformationEnergyChoreonoid
      (cnoid::BodyIMeshPtr mesh,
       const std::size_t nDofs,
       const std::size_t nFrames,
       const vector_t& initialJointsTrajectory,
       boost::shared_ptr<GenericDifferentiableFunction<T> > jointToMarker,
       boost::shared_ptr<JointToMarkerPositionChoreonoid<T> >
       jointToMarkerOrigin)
	throw (std::runtime_error)
	: GenericDifferentiableFunction<T>
	  (nFrames * (6 + mesh->bodyInfo (0).body->numJoints ()), 1,
	   "BodyLaplacianDeformationEnergyChoreonoid"),
	  mesh_ (mesh),
	  nDofs_ (nDofs),
	  nFrames_ (nFrames),
	  jointToMarker_ (jointToMarker),
	  jointToMarkerOrigin_ (jointToMarkerOrigin),
	  markerPositions_ (mesh_->numMarkers () * 3),

	  originalLaplacianCoordinates_
	  (mesh_->getNumFrames () * mesh_->numMarkers () * 3),
	  currentLaplacianCoordinates_
	  (mesh_->getNumFrames () * mesh_->numMarkers () * 3)
      {
	assert (mesh_->getNumFrames () == nFrames);

	// Compute initial Laplacian coordinates.
	for (int frameId = 0; frameId < nFrames_; ++frameId)
	  {
	    const cnoid::BodyIMesh::Frame& neighborLists =
	      mesh_->frame (frameId);
	    const cnoid::BodyIMesh::NeighborList&
	      neighbors = neighborLists[frameId];

	    // Compute the current marker position
	    (*jointToMarker_)
	      (markerPositions_,
	       initialJointsTrajectory.segment
	       (frameId * nDofs_, nDofs_));

	    // Compute Laplacian Coordinates of each marker.
	    originalLaplacianCoordinates_.segment
	      (frameId * mesh_->numMarkers () * 3,
	       mesh_->numMarkers () * 3) = markerPositions_;
	  }
      }

      virtual ~BodyLaplacianDeformationEnergyChoreonoid () throw ()
      {}

    protected:
      void
      impl_compute
      (result_t& result, const argument_t& x)
	const throw ()
      {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	jointToMarkerOrigin_->shouldUpdate ();

	result[0] = 0.;
	for (int frameId = 0; frameId < mesh_->getNumFrames (); ++frameId)
	  {
	    const cnoid::BodyIMesh::Frame& neighborLists =
	      mesh_->frame (frameId);
	    const cnoid::BodyIMesh::NeighborList&
	      neighbors = neighborLists[frameId];

	    // Compute the current marker position
	    jointToMarkerOrigin_->frameId () = frameId;
	    (*jointToMarker_)
	      (markerPositions_, x.segment (frameId * nDofs_, nDofs_));

	    // Compute Laplacian Coordinates of each markerq
	    currentLaplacianCoordinates_.segment
	      (frameId * mesh_->numMarkers () * 3,
	       mesh_->numMarkers () * 3) = markerPositions_;

	    for (int markerId = 0; markerId < mesh_->numMarkers (); ++markerId)
	      {
		for (int l = 0; l < neighbors.size (); ++l)
		  {
		    int neighborId = neighbors[l];
		    double weight =
		      (originalLaplacianCoordinates_.segment
		       (mesh_->numMarkers () * 3 + markerId * 3, 3) -
		       originalLaplacianCoordinates_.segment
		       (mesh_->numMarkers () * 3 + neighborId * 3, 3)).norm ();
		    weight = 1. / weight;
		    currentLaplacianCoordinates_.segment
		      (mesh_->numMarkers () * 3 + markerId * 3, 3) -=
		      weight *
		      markerPositions_.segment (neighborId * 3, 3);
		  }
	      }

	    currentLaplacianCoordinates_ -= originalLaplacianCoordinates_;
	    result[0] += currentLaplacianCoordinates_.norm ();
	  }
	result[0] *= .5;
      }

      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type i)
	const throw ()
      {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	roboptim::GenericFiniteDifferenceGradient<
	  T,
	  finiteDifferenceGradientPolicies::Simple<T> >
	  fdg (*this);
	fdg.gradient (gradient, x, i);
      }

    private:
      cnoid::BodyIMeshPtr mesh_;
      std::size_t nDofs_;
      std::size_t nFrames_;

      boost::shared_ptr<GenericDifferentiableFunction<T> > jointToMarker_;
      boost::shared_ptr<JointToMarkerPositionChoreonoid<T> >
      jointToMarkerOrigin_;
      mutable result_t markerPositions_;

      result_t originalLaplacianCoordinates_;
      mutable result_t currentLaplacianCoordinates_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_BODY_LAPLACIAN_DEFORMATION_ENERGY_CHOREONOID_HH
