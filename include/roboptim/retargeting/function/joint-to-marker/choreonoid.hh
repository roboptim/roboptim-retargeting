#ifndef ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_CHOREONOID_HH
# include <stdexcept>

# include <roboptim/core/differentiable-function.hh>
# include <roboptim/core/finite-difference-gradient.hh>

// For update configuration function.
# include <roboptim/retargeting/function/forward-geometry/choreonoid.hh>

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
	  markerPositions_ (mesh->numMarkers ()),
	  fd_ (boost::make_shared<fdFunction_t> (*this))
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
	assert (mesh_->numBodies () == 1);

	cnoid::BodyIMesh::BodyInfo& bodyInfo = mesh_->bodyInfo (0);
	cnoid::BodyPtr& body = bodyInfo.body;
	cnoid::Link* rootLink = body->rootLink ();

	std::size_t offset = 6;

	// Set the robot configuration.
	updateRobotConfiguration (body, x);

	// Update body positions
	body->calcForwardKinematics ();

	// combine forward geometry with marker offset
        const std::vector<cnoid::BodyIMesh::MarkerPtr>&
	  markers = bodyInfo.markers;

	std::size_t vertexIndex = 0;
        for(std::size_t j = 0; j < markers.size (); ++j)
	  {
            const cnoid::BodyIMesh::MarkerPtr& marker = markers[j];
            if(marker->localPos)
	      {
		result.segment (vertexIndex * 3, 3) = marker->link->p ();
		result.segment (vertexIndex * 3, 3) +=
		  marker->link->attitude () * (*marker->localPos);
	      }
	    else
	      result.segment (vertexIndex * 3, 3) = marker->link->p ();
	    ++vertexIndex;
	  }
      }

      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type i)
	const throw ()
      {
	fd_->gradient (gradient, x, i);
      }

    private:
      cnoid::BodyIMeshPtr mesh_;
      mutable std::vector<cnoid::Vector3> markerPositions_;

      // Temporary - finite difference gradient.
      typedef roboptim::GenericFiniteDifferenceGradient<
	T, finiteDifferenceGradientPolicies::Simple<T> >
      fdFunction_t;
      boost::shared_ptr<fdFunction_t> fd_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_CHOREONOID_HH
