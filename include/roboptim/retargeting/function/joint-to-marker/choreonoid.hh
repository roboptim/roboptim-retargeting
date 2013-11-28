#ifndef ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_CHOREONOID_HH
# include <stdexcept>

# include <roboptim/core/differentiable-function.hh>

# include <cnoid/BodyIMesh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Convert joint position into marker position.
    ///
    /// Input: [q]
    /// where q is the value of a particular joint
    /// [q] is the full robot configuration including the free-floating joint.
    ///
    /// Output: [m]
    /// where m is a marker position in the 3d space (x, y, z)
    /// [m] is the array of all markers associated with the character
    ///
    /// The joint/marker offset is constant.
    template <typename T>
    class JointToMarkerPositionChoreonoid : public GenericDifferentiableFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_ (GenericDifferentiableFunction<T>);

      explicit JointToMarkerPositionChoreonoid
      (cnoid::BodyIMeshPtr mesh, size_type frameId)
	throw (std::runtime_error)
	: GenericDifferentiableFunction<T>
	  (6 + mesh->bodyInfo (0).body->numJoints (), 3 * mesh->numMarkers (),
	   "JointToMarkerPosition"),
	  frameId_ (frameId),
	  shouldUpdate_ (true),
	  mesh_ (mesh),
	  markerPositions_ (mesh->numMarkers ())
      {
	if (mesh_->numBodies () != 1)
	  {
	    boost::format fmt
	      ("unexpected number of body when instantiating"
	       " JointToMarkerPositionChoreonoid"
	       " (expected is 1 but %d body are present)");
	    fmt % mesh_->numBodies ();
	    throw std::runtime_error (fmt.str ());
	  }
      }

      virtual ~JointToMarkerPositionChoreonoid () throw ()
      {
      }

      const size_type& frameId () const throw ()
      {
	return frameId_;
      }

      size_type& frameId () throw ()
      {
	return frameId_;
      }

      void shouldUpdate () throw ()
      {
	shouldUpdate_ = true;
      }

    protected:
      void
      impl_compute
      (result_t& result, const argument_t& x)
	const throw ()
      {
	assert (mesh_->numBodies () == 1);

	cnoid::BodyIMesh::BodyInfo& bodyInfo = mesh_->bodyInfo (0);
	cnoid::BodyMotionPtr motion = bodyInfo.motion;

	std::size_t offset = 6;
	std::size_t nDofs = offset + motion->numJoints ();

#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	if (shouldUpdate_)
	  {
	    if (frameId_ < motion->linkPosSeq ()->numFrames ())
	      {
		motion->linkPosSeq ()->frame (frameId_)[0].translation () =
		  x.segment (frameId_ * nDofs, 3);

		value_type norm = x.segment (frameId_ * nDofs + 3, 3).norm ();

		if (norm < 1e-10)
		  motion->linkPosSeq ()->frame
		    (frameId_)[0].rotation ().setIdentity ();
		else
		  motion->linkPosSeq ()->frame
		    (frameId_)[0].rotation () =
		    Eigen::AngleAxisd
		    (norm,
		     x.segment (frameId_ * nDofs + 3, 3).normalized ()
		     );
	      }

	      for (int dofId = 0; dofId < motion->numJoints (); ++dofId)
		motion->jointPosSeq ()->frame (frameId_)[dofId] =
		  x[frameId_ * nDofs + offset + dofId];

	      mesh_->update ();
	      mesh_->getVertices (markerPositions_);
	      shouldUpdate_ = false;
	  }

	for (int markerId = 0; markerId < mesh_->numMarkers (); ++markerId)
	  result.segment (markerId * 3, 3) = markerPositions_[markerId];

#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (false);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION
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
      size_type frameId_;
      mutable bool shouldUpdate_;
      cnoid::BodyIMeshPtr mesh_;
      mutable std::vector<cnoid::Vector3> markerPositions_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_CHOREONOID_HH
