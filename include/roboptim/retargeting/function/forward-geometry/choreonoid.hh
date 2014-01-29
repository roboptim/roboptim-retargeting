#ifndef ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_CHOREONOID_HH
# include <boost/format.hpp>

# include <cnoid/Body>
# include <cnoid/JointPath>
# include <cnoid/Jacobian>

# include <roboptim/retargeting/function/forward-geometry.hh>

# include <roboptim/core/finite-difference-gradient.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Update Choreonoid robot from configuration vector.
    ///
    /// It is a two step process:
    /// 1. extract the first 6 DOFs of the configuration vector
    ///    which represents the root link position.
    ///    The first three values are the translation part,
    ///    The next three values are the rotational part expressed
    ///    using the U-Theta representation.
    ///
    /// 2. The remaining values are the DOFs configurations, pass
    ///    them directly to Choreonoid.
    ///
    /// This function does not realize any allocation and can be
    /// safely used in function computations.
    ///
    /// \param robot Choreonoid robot
    /// \param x configuration vector which size must correspond to
    ///          the robot number of DOFs plus six.
    template <typename T>
    void
    updateRobotConfiguration (const cnoid::BodyPtr& robot,
			      const Eigen::MatrixBase<T>& x)
    {
      // Set root link position.
      robot->rootLink ()->position ().translation () = x.segment (0, 3);

      typename T::RealScalar norm = x.segment(3, 3).norm ();

      if (norm < 1e-10)
	robot->rootLink ()->position ().linear ().setIdentity ();
      else
	{
	  Eigen::Vector3d normalizedAxis = x.segment (3, 3);
	  normalizedAxis /= norm;

	  Eigen::AngleAxisd angleAxisBuffer
	    (norm, normalizedAxis);
	  robot->rootLink ()->position ().linear ().setIdentity ();
	  robot->rootLink ()->position ().rotate (angleAxisBuffer);
	}

      // Set joints values.
      for(int dofId = 0; dofId < robot->numJoints (); ++dofId)
	robot->joint (dofId)->q () = x[dofId];
    }

    /// \brief Compute forward geometry for a particular robot model.
    ///
    /// Robot model from choreonoid should be passed to the
    /// constructor and use for computation.  The analytical jacobian
    /// must also be retrieved and returned in the
    /// impl_gradient/impl_jacobian methods.
    template <typename T>
    class ForwardGeometryChoreonoid : public ForwardGeometry<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_ (ForwardGeometry<T>);

      explicit ForwardGeometryChoreonoid
      (cnoid::BodyPtr robot, int bodyId) throw ()
	: ForwardGeometry<T> (6 + robot->numJoints (), "choreonoid"),
	  robot_ (robot),
	  bodyId_ (bodyId),
	  jointPath_ (robot->rootLink ()),
	  angleAxis_ (),
	  J_ (),
	  fd_ (boost::make_shared<fdFunction_t> (*this))
      {
	if (bodyId >= robot->numLinks () || robot->link (bodyId_) == 0)
	  {
	    boost::format fmt
	      ("failed to construct ForwardGeometryChoreonoid function:"
	       " invalid body id %d (robot contains %d bodies)");
	    fmt % bodyId % robot->numLinks ();
	    throw std::runtime_error (fmt.str ());
	  }

	jointPath_.setPath (robot->rootLink (), robot->link (bodyId_));
	J_.resize(6, jointPath_.numJoints ());
      }

      explicit ForwardGeometryChoreonoid
      (cnoid::BodyPtr robot, const std::string& bodyName) throw ()
	: ForwardGeometry<T> (6 + robot->numJoints (), "choreonoid"),
	  robot_ (robot),
	  bodyId_ (0),
	  jointPath_ (robot->rootLink ()),
	  J_ (),
	  fd_ (boost::make_shared<fdFunction_t> (*this))
      {
	cnoid::Link* link = robot->link (bodyName.c_str ());
	if (!link)
	  {
	    boost::format fmt
	      ("failed to construct ForwardGeometryChoreonoid function:"
	       " no body whose name is '%s' can be found");
	    fmt % bodyName;
	    throw std::runtime_error (fmt.str ());
	  }

	bodyId_ = link->jointId ();
	if (bodyId_ >= robot->numLinks () || robot->link (bodyId_) == 0)
	  {
	    boost::format fmt
	      ("failed to construct ForwardGeometryChoreonoid function:"
	       " invalid body id %d (robot contains %d bodies)");
	    fmt % bodyId_ % robot->numLinks ();
	    throw std::runtime_error (fmt.str ());
	  }

	jointPath_.setPath (robot->rootLink (), robot->link (bodyId_));
	J_.resize(6, jointPath_.numJoints ());
      }


      virtual ~ForwardGeometryChoreonoid () throw ()
      {}

    protected:

      // FIXME: once again we make the assumption that all joints are
      // 1-DOFs joints.
      void
      impl_compute
      (result_t& result, const argument_t& x)
	const throw ()
      {
	// Set the robot configuration.
	updateRobotConfiguration (robot_, x);

	// Update positions.
	robot_->calcForwardKinematics ();

	// Convert rotation matrix to angle axis temporary buffer.
	angleAxis_.fromRotationMatrix (jointPath_.endLink ()->R ());

	// Copy the result back.
	result.segment (0, 3) = jointPath_.endLink ()->p ();
	result.segment (3, 3) = angleAxis_.angle () * angleAxis_.axis ();
      }


      /// \brief Gradient computation.
      ///
      /// The jacobian is as follow:
      ///
      /// ------------------------------------------------------------------
      /// | function id || tx | ty | tz | rx | ry | rz | dof0 | ... | dofN |
      /// ------------------------------------------------------------------
      /// | 0 (tx)      || 1  | 0  | 0  |              |                   |
      /// | 1 (ty)      || 0  | 1  | 0  |      ?       | IMPLEMENTED USING |
      /// | 2 (tz)      || 0  | 0  | 1  |              |    CHOREONOID     |
      /// | 3 (rx)      || 0  | 0  | 0  |              |                   |
      /// | 4 (ry)      || 0  | 0  | 0  |      ?       | ROTATION PART IS  |
      /// | 5 (rz)      || 0  | 0  | 0  |              | CURRENTLY WRONG   |
      /// ------------------------------------------------------------------
      ///
      /// I.e. influence of configuration change on the body position.
      ///
      /// In our case, we only compute one line of this array.
      ///
      ///
      /// Issues:
      /// -------
      ///
      /// - rotational part not known
      ///
      /// - rotation representations in RobOptim and Choreonoid are not
      ///   the same
      ///
      ///
      /// \param gradient Gradient
      /// \param x configuration (free floating then DOF values)
      /// \param functionId jacobian line to be computed
      void
      impl_gradient2 (gradient_t& gradient,
		     const argument_t& x,
		     size_type functionId)
	const throw ()
      {
	// Set the robot configuration.
	updateRobotConfiguration (robot_, x);

	// Update positions.
	robot_->calcForwardKinematics ();


#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	gradient.setZero ();

	// Free floating (columns 0 to 5).

	// 6x6 first block diagonal is identity
	gradient[functionId] = 1.;

	if (functionId < 3)
	  {
	    std::cout
	      << "result:\n"
	      << this->operator () (x) << '\n'
	      << "base link position:\n"
	      << jointPath_.baseLink ()->p () << '\n'
	      << "end link position:\n"
	      << jointPath_.endLink ()->p () << '\n'
	      << "base link rotation:\n"
	      << jointPath_.baseLink ()->R () << '\n'
	      << "end link rotation:\n"
	      << jointPath_.endLink ()->R () << '\n';


	    cnoid::Vector3 arm =
	      jointPath_.endLink ()->p () - jointPath_.baseLink ()->p ();
	    cnoid::Vector3 omega (3);
	    for (std::size_t i = 0; i < 3; ++i)
	      {
		omega.setZero ();
		omega[i] = 1.;
		gradient[3 + i] = omega.cross (arm)[functionId];
	      }
	  }

	// DOF (all columns > 5).

	// First we compute the jacobian for the current joint path.
	//jointPath_.calcJacobian (J_);
	cnoid::setJacobian<0x3f, 0, 0>
	  (this->jointPath_, this->jointPath_.endLink (), J_);

	// And we replace at the right position. The jacobian of the
	// joint path is incomplete so we have to copy the values to
	// the right location manually. All the joints which are not
	// in the joint path will not have any effect.
	std::size_t jointId = 0;
	for (std::size_t jacobianId = 0; jacobianId < jointPath_.numJoints (); ++jacobianId)
	  {
	    //FIXME: why should we remove one here !?
	    jointId = jointPath_.joint(jacobianId)->index () - 1;
	    assert (jointId < gradient.size ());
	    assert (jointId >= 6);

	    gradient[jointId] = J_ (functionId, jacobianId);
	  }
      }

      // temporary version using finite differences gradient.
      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type functionId)
	const throw ()
      {
	fd_->gradient (gradient, x, functionId);
      }

    private:
      cnoid::BodyPtr robot_;
      int bodyId_;


      mutable cnoid::JointPath jointPath_;
      /// \brief Root link position
      mutable cnoid::Position rootLinkPosition_;
      /// \brief Temporary variable used for angle conversions.
      mutable Eigen::AngleAxis<value_type> angleAxis_;

      /// \brief Jacobian computed by Choreonoid (buffer).
      mutable cnoid::MatrixXd J_;


      // Temporary - finite difference gradient.
      typedef roboptim::GenericFiniteDifferenceGradient<
	T, finiteDifferenceGradientPolicies::Simple<T> >
      fdFunction_t;
      boost::shared_ptr<fdFunction_t> fd_;

    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_CHOREONOID_HH
