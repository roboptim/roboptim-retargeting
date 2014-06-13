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

#ifndef ROBOPTIM_RETARGETING_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_CHOREONOID_HH
# include <cnoid/Body>

# include <roboptim/core/function.hh> //FIXME: Eigen

# include <roboptim/retargeting/eigen-rigid-body.hh>
# include <roboptim/retargeting/utility.hh>

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
    template <typename Derived>
    void
    updateRobotConfiguration (const cnoid::BodyPtr& robot,
			      const Eigen::MatrixBase<Derived>& x);

    template <typename Derived>
    void
    updateRobotConfiguration (const cnoid::BodyPtr& robot,
			      const Eigen::MatrixBase<Derived>& x)
    {
      // Set root link position.
      robot->rootLink ()->position ().translation () =
	x.template segment<3> (0);
      eulerToTransform
	(robot->rootLink ()->position ().linear (), x.template segment<3> (3));

      // Set joints values.
      for(int dofId = 0; dofId < robot->numJoints (); ++dofId)
	robot->joint (dofId)->q () = x[dofId + 6];
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_CHOREONOID_HH
