#ifndef ROBOPTIM_RETARGETING_FUNCTION_TORQUE_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FUNCTION_TORQUE_CHOREONOID_HH
# include <roboptim/core/finite-difference-gradient.hh>

# include <cnoid/Body>
# include <cnoid/ForwardDynamics>

# include <roboptim/retargeting/function/torque.hh>

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
	  forwardDynamics_ ()
      {}

      virtual ~TorqueChoreonoid () throw ()
      {}

      void
      impl_compute
      (result_t& result, const argument_t& x)
	const throw ()
      {
	// Set root link position.
	rootLinkPosition_.translation () = this->translation (x);

#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	value_type norm = this->rotation (x).norm ();

	if (norm < 1e-10)
	  rootLinkPosition_.linear ().setIdentity ();
	else
	  rootLinkPosition_.linear () =
	    Eigen::AngleAxisd
	    (norm,
	     this->rotation (x).normalized ()).toRotationMatrix ();

#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (false);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	robot_->rootLink ()->position () = rootLinkPosition_;

	// Set q, dq, ddq.
	for(int dofId = 0; dofId < robot_->numJoints (); ++dofId)
	  {
	    robot_->joint (dofId)->q () = this->q (x, false)[dofId];
	    robot_->joint (dofId)->dq () = this->dq (x, false)[dofId];
	    robot_->joint (dofId)->ddq () = this->ddq (x, false)[dofId];
	  }
	robot_->calcForwardKinematics (true, true);
	result.setZero ();

	//FIXME:
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
      /// \brief Pointer to loaded robot model
      cnoid::BodyPtr robot_;

      /// \brief Root link position
      mutable cnoid::Position rootLinkPosition_;

      /// \brief Choreonoid Forward Dynamics object implementing Roy
      ///        Featherstone's articulated body algorithm.
      boost::shared_ptr<cnoid::ForwardDynamics> forwardDynamics_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_TORQUE_CHOREONOID_HH
