// Copyright (C) 2014 by Thomas Moulard, AIST, CNRS.
//
// This file is part of the roboptim.
//
// roboptim is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// roboptim is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with roboptim.  If not, see <http://www.gnu.org/licenses/>.

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

      explicit TorqueChoreonoid (cnoid::BodyPtr robot)
	// Add a fictional free floating joint at the beginning.
	: Torque<T> (6 + robot->numJoints (), "choreonoid"),
	  robot_ (robot),
	  forwardDynamics_ (),
	  fd_ (boost::make_shared<fdFunction_t> (*this))
      {}

      virtual ~TorqueChoreonoid ()
      {}

      void
      impl_compute
      (result_t& result, const argument_t& x)
	const
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
	const
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
