#ifndef ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_CHOREONOID_HH
# include <stdexcept>

# include <roboptim/core/differentiable-function.hh>
# include <roboptim/core/finite-difference-gradient.hh>

# include <cnoid/BodyIMesh>
# include <cnoid/Link>

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
      (cnoid::BodyIMeshPtr mesh)
	throw (std::runtime_error)
	: GenericDifferentiableFunction<T>
	  (6 + mesh->bodyInfo (0).body->numJoints (), 3 * mesh->numMarkers (),
	   "JointToMarkerPosition"),
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

	assert (mesh_->numBodies () == 1);

	cnoid::BodyIMesh::BodyInfo& bodyInfo = mesh_->bodyInfo (0);
	cnoid::BodyPtr& body = bodyInfo.body;
	cnoid::Link* rootLink = body->rootLink ();

	std::size_t offset = 6;

	// Update q in choreonoid model.

	// free floating
	rootLink->p() = x.segment (0, 3);

	value_type norm = x.segment (3, 3).norm ();
	if (norm < 1e-10)
	  rootLink->R ().setIdentity ();
	else
	  rootLink->R () =
	    Eigen::AngleAxisd
	    (norm,
	     x.segment (3, 3).normalized ()
	     ).toRotationMatrix ();

	// joints
	for (int dofId = 0; dofId < body->numJoints (); ++dofId)
	  body->joint (dofId)->q () = x[offset + dofId];

	// compute forward kinematics
	body->calcForwardKinematics(true, true);

	// combine forward geometry with marker offset
        const std::vector<cnoid::BodyIMesh::MarkerPtr>&
	  markers = bodyInfo.markers;

	std::size_t vertexIndex = 0;
        for(std::size_t j = 0; j < markers.size (); ++j)
	  {
            const cnoid::BodyIMesh::MarkerPtr& marker = markers[j];
            if(marker->localPos)
	      result.segment (vertexIndex * 3, 3) =
		marker->link->p ()
		+ marker->link->attitude () * (*marker->localPos);
            else
	      result.segment (vertexIndex * 3, 3) = marker->link->p ();
            ++vertexIndex;
	  }

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
      cnoid::BodyIMeshPtr mesh_;
      mutable std::vector<cnoid::Vector3> markerPositions_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_CHOREONOID_HH
