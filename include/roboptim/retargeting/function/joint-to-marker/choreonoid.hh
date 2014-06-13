#ifndef ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_CHOREONOID_HH
# include <stdexcept>

# include <libmocap/marker-trajectory.hh>

# include <roboptim/core/differentiable-function.hh>

# include <roboptim/retargeting/morphing.hh>

// For update configuration function.
# include <roboptim/retargeting/function/forward-geometry/choreonoid.hh>

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
      (cnoid::BodyPtr robot,
       const MorphingData& morphing)
	: GenericDifferentiableFunction<T>
	  (6 + robot->numJoints (),
	   3 * static_cast<size_type> (morphing.markers.size ()),
	   "JointToMarkerPosition"),
	  robot_ (robot),
	  morphing_ (morphing),
	  markerPositions_
	  (static_cast<std::size_t> (3 * morphing.markers.size ())),
	  jointPath_ (),
	  J_ (),
	  dR_ ()
      {
	J_.resize (3, robot->numJoints ());
	dR_[0].setZero ();
	dR_[1].setZero ();
	dR_[2].setZero ();
      }

      virtual ~JointToMarkerPositionChoreonoid ()
      {}

    protected:
      void
      impl_compute
      (result_t& result, const argument_t& x)
	const
      {
	// Set the robot configuration.
	updateRobotConfiguration (robot_, x);

	// Update body positions
	robot_->calcForwardKinematics ();

	// combine forward geometry with marker offset
	typedef std::vector<std::string>::const_iterator const_iterator;

	typename result_t::Index markerIndex = 0;
	for (const_iterator it = morphing_.markers.begin ();
	     it != morphing_.markers.end ();
	     ++it, ++markerIndex)
	  {
	    try
	      {
		const std::string& linkName = morphing_.attachedBody (*it);
		cnoid::Link* link = robot_->link (linkName);

		result.segment (markerIndex * 3, 3) = link->p ();
		result.segment (markerIndex * 3, 3) +=
		  morphing_.offset (linkName, *it);
	      }
	    catch (const std::exception& e)
	      {
		result.segment (markerIndex * 3, 3).setZero ();
	      }
	  }
      }

      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type functionId)
	const
      {
	cnoid::Link* rootLink = robot_->rootLink ();

	// Determine which marker matches this
	//
	// dim means which dimension (0 is x, 1 is y, 2 is z)
	//
	// markerId is the marker index that has to be considered
	typename gradient_t::Index dim = functionId % 3;
	std::size_t markerId =
	  static_cast<std::size_t> (functionId - dim) / 3;

	gradient.template segment<3> (0).setZero ();
	gradient[dim] = 1.;

	// look for marker information
	const std::string& markerName = morphing_.markers[markerId];
	cnoid::Link* link;
	std::string linkName;
	try
	  {
	    linkName = morphing_.attachedBody (markerName);
	    link = robot_->link (linkName);
	  }
	catch (const std::exception& e)
	  {
	    gradient.setZero ();
	    return;
	  }

	// Update paths.
	jointPath_.setPath (rootLink, link);

	// Set the robot configuration.
	updateRobotConfiguration (robot_, x);

	// Update body positions
	robot_->calcForwardKinematics ();

	// Compute the jacobian.
	Eigen::Vector3d localPos =
	  morphing_.offset (linkName, markerName);

	cnoid::setJacobian<0x7, 0, 0, true>
	  (jointPath_, link, localPos, J_);

	const typename cnoid::Position::LinearPart& R0 =
	  jointPath_.baseLink ()->T ().linear ();
	const typename cnoid::Position::LinearPart& R =
	  jointPath_.endLink ()->T ().linear ();
	const typename cnoid::Position::TranslationPart& t0 =
	  jointPath_.baseLink ()->T ().translation ();
	const typename cnoid::Position::TranslationPart& tk =
	  jointPath_.endLink ()->T ().translation ();

	updateDR (x.template segment<3> (3));

	Eigen::Matrix<value_type, 3, 3> J_global =
	  R0.col (2) * R0.col (1).transpose () * dR_[0] +
	  R0.col (1) * R0.col (0).transpose () * dR_[2] +
	  R0.col (0) * R0.col (2).transpose () * dR_[1];

	Eigen::Matrix<value_type, 3, 1> p = tk + R * localPos - t0;
	Eigen::Matrix<value_type, 3, 3> hatp;
	hat (hatp, p);

	gradient.template segment<3> (3) = (-hatp * J_global).row (dim);

	// DOF (all columns > 5).

	// First we compute the jacobian for the current joint path.

	// And we replace at the right position. The jacobian of the
	// joint path is incomplete so we have to copy the values to
	// the right location manually. All the joints which are not
	// in the joint path will not have any effect.
	int jointId = 0;
	for (int jacobianId = 0;
	     jacobianId < jointPath_.numJoints (); ++jacobianId)
	  {
	    jointId = jointPath_.joint (jacobianId)->index () + 6 - 1;
	    ROBOPTIM_RETARGETING_ASSERT (jointId < gradient.size ());
	    ROBOPTIM_RETARGETING_ASSERT (jointId >= 6);

	    gradient[jointId] = J_ (dim, jacobianId);
	  }

      }

      void
      impl_jacobian (jacobian_t& jacobian,
		     const argument_t& x)
	const
      {
	cnoid::Link* rootLink = robot_->rootLink ();
	jacobian.setZero ();

	// Set the robot configuration.
	updateRobotConfiguration (robot_, x);

	// Update body positions
	robot_->calcForwardKinematics ();

	for (std::size_t markerId = 0;
	     markerId < morphing_.markers.size (); ++markerId)
	{
	  typename jacobian_t::Index markerId_ =
	    static_cast<typename jacobian_t::Index> (markerId);

	  jacobian.template block<3, 3> (markerId_ * 3, 0).setIdentity ();

	  // Update paths.
	  std::string markerName;
	  std::string linkName;
	  cnoid::Link* link;

	  try
	    {
	      markerName = morphing_.markers[markerId];
	      linkName = morphing_.attachedBody (markerName);
	      link = robot_->link (linkName);
	    }
	  catch (const std::exception&)
	    {
	      jacobian.template block<3, 3> (markerId_ * 3, 3).setZero ();
	      continue;
	    }


	  jointPath_.setPath (rootLink, link);

	  // Compute the jacobian.
	  Eigen::Vector3d localPos =
	    morphing_.offset (linkName, markerName);

	  cnoid::setJacobian<0x7, 0, 0, true>
	    (jointPath_, link, localPos, J_);

	  const typename cnoid::Position::LinearPart& R0 =
	    jointPath_.baseLink ()->T ().linear ();
	  const typename cnoid::Position::LinearPart& R =
	    jointPath_.endLink ()->T ().linear ();
	  const typename cnoid::Position::TranslationPart& t0 =
	    jointPath_.baseLink ()->T ().translation ();
	  const typename cnoid::Position::TranslationPart& tk =
	    jointPath_.endLink ()->T ().translation ();

	  updateDR (x.template segment<3> (3));

	  Eigen::Matrix<value_type, 3, 3> J_global =
	    R0.col (2) * R0.col (1).transpose () * dR_[0] +
	    R0.col (1) * R0.col (0).transpose () * dR_[2] +
	    R0.col (0) * R0.col (2).transpose () * dR_[1];

	  Eigen::Matrix<value_type, 3, 1> p = tk + R * localPos - t0;
	  Eigen::Matrix<value_type, 3, 3> hatp;
	  hat (hatp, p);

	  jacobian.template block<3, 3> (markerId_ * 3, 3) = -hatp * J_global;

	  // DOF (all columns > 5).

	  // First we compute the jacobian for the current joint path.

	  // And we replace at the right position. The jacobian of the
	  // joint path is incomplete so we have to copy the values to
	  // the right location manually. All the joints which are not
	  // in the joint path will not have any effect.
	  int jointId = 0;
	  for (int jacobianId = 0;
	       jacobianId < jointPath_.numJoints (); ++jacobianId)
	    {
	      jointId = jointPath_.joint (jacobianId)->index () + 6 - 1;
	      ROBOPTIM_RETARGETING_ASSERT (jointId < jacobian.cols ());
	      ROBOPTIM_RETARGETING_ASSERT (jointId >= 6);

	      jacobian.template block<3, 1> (markerId_ * 3, jointId) =
		J_.col (jacobianId);
	    }
	}
      }

      // See doc/sympy/euler-angles.py
      template <typename Derived>
      void updateDR (const typename Eigen::MatrixBase<Derived>& x) const
      {
	value_type cr, sr, cp, sp, cy, sy;
	sincos (x[0], &sr, &cr);
	sincos (x[1], &sp, &cp);
	sincos (x[2], &sy, &cy);

	dR_[0] <<
	  0.,  -cy * sp,  -sy * cp,
	  0.,  -sy * sp,  cy * cp,
	  0.,  -cp,       0.;

	dR_[1] <<
	  cy * sp * cr + sy * sr,
	  cy * cp * sr,
	  -sy * sp * sr - cy * cr,

	  sy * sp * cr - cy * sr,
	  sy * cp * sr,
	  cy * sp * sr - sy * cr,

	  cp * cr, -sp * sr, 0.;

	dR_[2] <<
	  -cy * sp * sr + sy * cr,
	  cy * cp * cr,
	  -sy * sp * cr + cy * sr,

	  - sy * sp * sr - cy * cr,
	  sy * cp * cr,
	  cy * sp * cr + sy * sr,

	  -cp * sr, -sp * cr, 0.;
      }

    private:
      cnoid::BodyPtr robot_;
      const MorphingData& morphing_;

      mutable std::vector<
	cnoid::Vector3,
	Eigen::aligned_allocator<cnoid::Vector3>
	>
	markerPositions_;

      mutable cnoid::JointPath jointPath_;
      mutable cnoid::MatrixXd J_;
      mutable boost::array<Eigen::Matrix<value_type, 3, 3>, 3> dR_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_CHOREONOID_HH
