#ifndef ROBOPTIM_RETARGETING_FUNCTION_ZMP_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FUNCTION_ZMP_CHOREONOID_HH
# include <roboptim/core/finite-difference-gradient.hh>

# include <cnoid/Body>

# include <roboptim/retargeting/function/zmp.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief ZMP position computed through Choreonoid
    ///
    /// \tparam T Function traits type
    template <typename T>
    class ZMPChoreonoid : public ZMP<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_ (ZMP<T>);

      explicit ZMPChoreonoid (cnoid::BodyPtr robot) throw ()
	: ZMP<T> (robot->numJoints (), "choreonoid"),
	  robot_ (robot),
	  g_ (9.81)
      {}

      virtual ~ZMPChoreonoid () throw ()
      {}

    protected:
      void
      impl_compute
      (result_t& result, const argument_t& x)
	const throw ()
      {
	for(std::size_t dofId = 0; dofId < robot_->numJoints (); ++dofId)
	  {
	    robot_->joint (dofId)->q () =
	      x.segment (0 * robot->numJoints ())[dofId];
	    robot_->joint (dofId)->dq () =
	      x.segment (1 * robot->numJoints ())[dofId];
	    robot_->joint (dofId)->ddq () =
	      x.segment (2 * robot->numJoints ())[dofId];
	  }
	robot_->calcForwardKinematics (true, true);
	const cnoid::Vector3& com = robot_->calcCenterOfMass ();
	const double& m = robot_->mass ();
	cnoid::Vector3 P;
	cnoid::Vector3 L;
	robot_->calcTotalMomentum (P, L);

	//FIXME: to be computed by fd.
	cnoid::Vector3 ddcom; // \ddot{x}
	cnoid::Vector3 dL; // \dot{L}

	// Reorder dL
	cnoid::Vector2 dL_reordered;
	dL_reordered[0] = dL[1];
	dL_reordered[1] = dL[0];

	// alpha = \ddot{x_z} + g
	const double alpha = ddcom[2] + g_;

	result = com.segment<2> (0);
	result -= dL_reordered / (m * alpha);
	result -= ddcom.segment<2> (0) * (com[2] / alpha);
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
      cnoid::BodyPtr robot_;
      value_type g_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_ZMP_HH
