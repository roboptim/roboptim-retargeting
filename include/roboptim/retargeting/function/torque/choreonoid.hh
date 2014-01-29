#ifndef ROBOPTIM_RETARGETING_FUNCTION_TORQUE_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FUNCTION_TORQUE_CHOREONOID_HH
# include <roboptim/core/finite-difference-gradient.hh>

# include <cnoid/Body>
# include <cnoid/ForwardDynamics>

# include <roboptim/retargeting/function/torque.hh>

// For update configuration function.
# include <roboptim/retargeting/function/forward-geometry/choreonoid.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Torque position computed through Choreonoid
    ///
    /// \tparam T Function traits type
    template <typename T>
    class TorqueChoreonoid : public Torque<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_ (Torque<T>);

      explicit TorqueChoreonoid (cnoid::BodyPtr robot) throw ()
	// Add a fictional free floating joint at the beginning.
	: Torque<T> (6 + robot->numJoints (), "choreonoid"),
	  robot_ (robot),
	  forwardDynamics_ (),
	  fd_ (boost::make_shared<fdFunction_t> (*this))
      {}

      virtual ~TorqueChoreonoid () throw ()
      {}

      void
      impl_compute
      (result_t& result, const argument_t& x)
	const throw ()
      {
	// Set the robot configuration.
	updateRobotConfiguration (robot_, x);

	// Set dq, ddq.
	for(int dofId = 0; dofId < robot_->numJoints (); ++dofId)
	  {
	    robot_->joint (dofId)->q () = this->q (x, false)[dofId];
	    robot_->joint (dofId)->dq () = this->dq (x, false)[dofId];
	    robot_->joint (dofId)->ddq () = this->ddq (x, false)[dofId];
	  }

	result.setZero ();

	//FIXME:
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
      /// \brief Pointer to loaded robot model
      cnoid::BodyPtr robot_;

      /// \brief Root link position
      mutable cnoid::Position rootLinkPosition_;

      /// \brief Choreonoid Forward Dynamics object implementing Roy
      ///        Featherstone's articulated body algorithm.
      boost::shared_ptr<cnoid::ForwardDynamics> forwardDynamics_;

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

#endif //! ROBOPTIM_RETARGETING_FUNCTION_TORQUE_CHOREONOID_HH
