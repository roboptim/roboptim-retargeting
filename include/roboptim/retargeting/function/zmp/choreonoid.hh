#ifndef ROBOPTIM_RETARGETING_FUNCTION_ZMP_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FUNCTION_ZMP_CHOREONOID_HH
# include <boost/array.hpp>

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

      struct State
      {
	vector_t x;
	cnoid::Vector3 com;
	cnoid::Vector3 P;
	cnoid::Vector3 L;
      };


      explicit ZMPChoreonoid (cnoid::BodyPtr robot) throw ()
	// Add a fictional free floating joint at the beginning.
	: ZMP<T> (6 + robot->numJoints (), "choreonoid"),
	  robot_ (robot),
	  g_ (9.81),
	  delta_ (1e-5),
	  m_ (robot->mass ()),
	  states_ ()
      {
	for (std::size_t i = 0; i < states_.size (); ++i)
	  states_[i].x.resize (6 + robot->numJoints ());
      }

      virtual ~ZMPChoreonoid () throw ()
      {}

      const State& states () const throw ()
      {
	return states_;
      }

      void printState (std::ostream& o, std::size_t i) const throw ()
      {
	if (i >= states_.size ())
	  {
	    o << "State does not exist" << decindent;
	    return;
	  }
	o << "x: " << incindent << iendl
	  << states_[i].x << decindent << iendl
	  << "CoM: " << states_[i].com << iendl
	  << "P: " << states_[i].P << iendl
	  << "L: " << states_[i].L;
      }

      void printQuantities (std::ostream& o) const throw ()
      {
	o << "g: " << g_ << iendl
	  << "delta: " << delta_ << iendl
	  << "m: " << m_ << iendl
	  << "States: " << incindent << iendl;
	for (std::size_t i = 0; i < states_.size (); ++i)
	  {
	    o << "State " << i << incindent << iendl;
	    printState (o, i);
	    if (i == states_.size () - 1)
	      o << decindent;
	    o << decindent << iendl;
	  }
	o  << "CoM acceleration: " << incindent << iendl
	   << ddcom_ << decindent << iendl
	   << "Variation of the kinetic momentum:" << incindent << iendl
	   << dL_ << decindent << iendl
	   << "Reordered variation of the kinetic momentum:" << incindent << iendl
	   << dL_reordered_ << decindent << iendl;
      }

    protected:
      void fillStates (const argument_t& x) const throw ()
      {
	// Store q
	states_[0].x = this->q (x, true);
	assert (states_[0].x.size () == x.size () / 3);

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
	states_[0].com = robot_->calcCenterOfMass ();
	robot_->calcTotalMomentum (states_[0].P, states_[0].L);

	for (std::size_t i = 1; i < states_.size (); ++i)
	  {
	    // Store q + delta * dq
	    states_[i].x = states_[i - 1].x;
	    states_[i].x += delta_ * this->dq (x);

	    // Update robot position (dq, ddq are unchanged).
	    for(int dofId = 0; dofId < robot_->numJoints (); ++dofId)
	      robot_->joint (dofId)->q () = states_[i].x[dofId];

	    // Compute quantities
	    robot_->calcForwardKinematics (true, true);

	    // Store CoM
	    states_[i].com = robot_->calcCenterOfMass ();

	    // Compute momentum and store it
	    robot_->calcTotalMomentum (states_[i].P, states_[i].L);
	  }
      }

      void
      impl_compute
      (result_t& result, const argument_t& x)
	const throw ()
      {
	fillStates (x);

	// Compute center of mass acceleration
	ddcom_ = states_[2].com - (2 * states_[1].com) + states_[0].com;
	ddcom_ /= delta_ * delta_;

	// Compute variation of the kinetic momentum
	dL_ = (states_[1].L - states_[0].L) / delta_;

	// Reorder dL
	dL_reordered_[0] = dL_[1];
	dL_reordered_[1] = dL_[0];

	// alpha = \ddot{x_z} + g
	const double alpha = ddcom_[2] + g_;

	result = this->states_[0].com.segment (0, 2);
	result -= dL_reordered_ / (m_ * alpha);
	result -= ddcom_.segment<2> (0) * (states_[0].com[2] / alpha);
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
      /// \brief Gravitational constant
      value_type g_;
      /// \brief Delta used for finite differentiation
      value_type delta_;
      /// \brief Robot total mass
      value_type m_;

      /// \brief Root link position
      mutable cnoid::Position rootLinkPosition_;

      /// \brief Set of computed quantities for
      ///        q, q + delta, q + 2 * delta
      mutable boost::array<State, 3> states_;

      /// \brief Center of mass acceleration.
      ///
      /// \ddot{x}
      mutable cnoid::Vector3 ddcom_;

      /// \brief Variation of the kinetic momentum around the center
      ///        of mass.
      ///
      /// \dot{L}
      mutable cnoid::Vector3 dL_;

      /// \brief The equation need dL in another order, so reorder and
      ///        store it here for direct use in the final expression.
      ///
      /// [ \dot{\delta_y}, \dot{\delta_x}]
      mutable cnoid::Vector2 dL_reordered_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_ZMP_HH
