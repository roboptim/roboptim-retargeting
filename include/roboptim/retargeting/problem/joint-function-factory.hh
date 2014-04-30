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

#ifndef ROBOPTIM_RETARGETING_JOINT_FUNCTION_FACTORY_HH
# define  ROBOPTIM_RETARGETING_JOINT_FUNCTION_FACTORY_HH
# include <boost/shared_ptr.hpp>

# include <roboptim/trajectory/trajectory.hh>

# include <roboptim/retargeting/problem/function-factory.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief POD structure containing all problems data.
    ///
    /// This gathers together several information from which
    /// functions parameters are set.
    struct JointFunctionData
    {
      /// \brief RobOptim trajectory
      boost::shared_ptr<roboptim::Trajectory<3> > trajectory;
    };

    /// \brief Creates functions from their name for joint-joint
    ///        optimization
    ///
    ///
    ///
    class JointFunctionFactory : public FunctionFactory
    {
    public:
      /// \brief  Constructor and Destructor
      /// \{

      /// \brief Constructor
      ///
      /// \param problem data (trajectories, etc.)
      explicit JointFunctionFactory (const JointFunctionData& data);

      ~JointFunctionFactory ();

      /// \}

      /// \brief Instantiate a function from its name.
      ///
      /// This method creates a function (cost or constraint) from its
      /// name.
      ///
      /// \param[in] name function name
      /// \return shared pointer to the newly created function
      /// \tparam T function type (e.g. DifferentiableFunction)
      template <typename T>
      boost::shared_ptr<T> buildFunction (const std::string& name);

      /// \brief Instantiate a constraint from its name.
      ///
      /// This method creates a constraint object from the constraint
      /// name.
      ///
      /// \param[in] name function name
      /// \return newly created constraint object
      /// \tparam T function type (e.g. DifferentiableFunction)
      template <typename T>
      Constraint<T> buildConstraint (const std::string& name);
    private:
      /// \brief Problem data (trajectories, etc.)
      const JointFunctionData& data_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

# include <roboptim/retargeting/problem/joint-function-factory.hxx>
#endif //! ROBOPTIM_RETARGETING_JOINT_FUNCTION_FACTORY_HH
