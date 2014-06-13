#ifndef ROBOPTIM_RETARGETING_FUNCTION_ZMP_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FUNCTION_ZMP_CHOREONOID_HH
# include <boost/array.hpp>

# include <roboptim/core/finite-difference-gradient.hh>

# include <cnoid/Body>

# include <roboptim/retargeting/function/zmp.hh>

// For update configuration function.
# include <roboptim/retargeting/function/forward-geometry/choreonoid.hh>

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


      explicit ZMPChoreonoid (cnoid::BodyPtr robot)
	// Add a fictional free floating joint at the beginning.
	: ZMP<T> (6 + robot->numJoints (), "choreonoid"),
	  robot_ (robot),
	  g_ (9.81),
	  delta_ (1e-5),
	  m_ (robot->mass ()),
	  states_ (),
	  fd_ (boost::make_shared<fdFunction_t> (*this))
      {
	for (std::size_t i = 0; i < states_.size (); ++i)
	  states_[i].x.resize (6 + robot->numJoints ());
      }

      explicit ZMPChoreonoid (const ZMPChoreonoid<T>& zmp)
	: ZMP<T> (6 + zmp.robot_->numJoints (), "choreonoid"),
	  robot_ (zmp.robot_),
	  g_ (zmp.g_),
	  delta_ (zmp.delta_),
	  m_ (zmp.m_),
	  states_ (zmp.states_),
	  fd_ (boost::make_shared<fdFunction_t> (*this))
      {
	for (std::size_t i = 0; i < states_.size (); ++i)
	  states_[i].x.resize (6 + robot_->numJoints ());
      }

      ZMPChoreonoid<T>& operator= (const ZMPChoreonoid<T>& rhs)
      {
	if (this == &rhs)
	  return *this;

	this->robot_ = rhs.robot_;
	this->g_ = rhs.g_;
	this->delta_ = rhs.delta_;
	this->m_ = rhs.m_;
	this->states_ = rhs.states_;
	this->fd_ = boost::make_shared<fdFunction_t> (*this);
	return *this;
      }

      virtual ~ZMPChoreonoid ()
      {}

      const State& states () const
      {
	return states_;
      }

      void printState (std::ostream& o, std::size_t i) const
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

      void printQuantities (std::ostream& o) const
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
      void fillStates (const argument_t& x) const
      {
	// Store q
	states_[0].x = this->q (x, true);
	assert (states_[0].x.size () == x.size () / 3);

	// Set the robot configuration.
	updateRobotConfiguration (robot_, x);

	// Set dq, ddq.
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
	    // Store q + delta * dq + .5 * ddq + delta^2
	    states_[i].x = states_[i - 1].x;
	    states_[i].x += delta_ * this->dq (x);
	    states_[i].x += .5 * this->ddq (x) * delta_ * delta_;

	    // Update robot position (dq, ddq are unchanged).
	    for(int dofId = 0; dofId < robot_->numJoints (); ++dofId)
	      robot_->joint (dofId)->q () = states_[i].x[6 + dofId];

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
	const
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
	const
      {
	fd_->gradient (gradient, x, i);
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
      /// \f$ \dot{L} \f$
      mutable cnoid::Vector3 dL_;

      /// \brief The equation need dL in another order, so reorder and
      ///        store it here for direct use in the final expression.
      ///
      /// \f$ [ \dot{\delta_y}, \dot{\delta_x}] \f$
      mutable cnoid::Vector2 dL_reordered_;

      // Temporary - finite difference gradient.
      // Currently the simple policy generates precision errors,
      // one may want to switch to five point rules.
      typedef roboptim::GenericFiniteDifferenceGradient<
	T, finiteDifferenceGradientPolicies::Simple<T> >
      fdFunction_t;
      boost::shared_ptr<fdFunction_t> fd_;


    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_ZMP_HH
