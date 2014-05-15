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

#ifndef ROBOPTIM_RETARGETING_MARKER_TO_JOINT_FUNCTION_FACTORY_HH
# define  ROBOPTIM_RETARGETING_MARKER_TO_JOINT_FUNCTION_FACTORY_HH
# include <boost/optional.hpp>
# include <boost/shared_ptr.hpp>

# include <cnoid/BodyMotion>

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
    struct MarkerToJointFunctionData
    {
      /// \brief marker set loaded by libmocap
      libmocap::MarkerSet markerSet;

      /// \brief markers trajectory as loaded by libmocap
      libmocap::MarkerTrajectory markersTrajectory;

      /// \brief Robot model loaded through Choreonoid
      ///
      /// Robot model contains the robot description of joints and
      /// bodies associated with limits such as joints positions,
      /// velocities limits, etc.
      cnoid::BodyPtr robotModel;

      /// \brief RobOptim markers trajectory
      boost::shared_ptr<roboptim::Trajectory<3> > inputTrajectory;

      /// \brief RobOPtim joints trajectory to be generated
      boost::shared_ptr<roboptim::Trajectory<3> > outputTrajectoryReduced;

      /// \brief RobOPtim joints trajectory to be generated
      boost::shared_ptr<roboptim::Trajectory<3> > outputTrajectory;

      boost::shared_ptr<DifferentiableFunction> cost;

      /// \brief Configuration of the disabled joints (one frame)
      ///
      ///
      std::vector<boost::optional<Function::value_type> >
      disabledJointsConfiguration;

      /// \brief Interaction Mesh (loaded by Choreonoid)
      cnoid::BodyIMeshPtr interactionMesh;

      Function::vector_t::Index frameId;


      Function::vector_t::Index nDofsFull () const
      {
	return
	  static_cast<Function::vector_t::Index>
	  (this->outputTrajectory->outputSize ());
      }
      Function::vector_t::Index nDofsFiltered () const
      {
	return
	  static_cast<Function::vector_t::Index>
	  (this->outputTrajectoryReduced->outputSize ());
      }

      Function::vector_t::Index nParametersFull () const
      {
	return
	  static_cast<Function::vector_t::Index>
	  (this->outputTrajectory->parameters ().size ());
      }
      Function::vector_t::Index nParametersFiltered () const
      {
	return
	  static_cast<Function::vector_t::Index>
	  (this->outputTrajectoryReduced->parameters ().size ());
      }
      Function::vector_t::Index nFrames () const
      {
	return
	  static_cast<Function::vector_t::Index>
	  (this->outputTrajectory->parameters ().size ())
	  / this->nDofsFull ();
      }

    };

    /// \brief Creates functions from their name for joint-joint
    ///        optimization
    ///
    ///
    ///
    class MarkerToJointFunctionFactory : public FunctionFactory
    {
    public:
      /// \brief  Constructor and Destructor
      /// \{

      /// \brief Constructor
      ///
      /// \param problem data (trajectories, etc.)
      explicit MarkerToJointFunctionFactory
      (const MarkerToJointFunctionData& data);

      ~MarkerToJointFunctionFactory ();

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

      /// \brief List supported functions.
      ///
      /// Each element of this string can be passed to buildFunction
      /// method to create the corresponding function.
      static std::vector<std::string>
      listFunctions ();
    private:
      /// \brief Problem data (trajectories, etc.)
      const MarkerToJointFunctionData& data_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

# include <roboptim/retargeting/problem/marker-to-joint-function-factory.hxx>
#endif //! ROBOPTIM_RETARGETING_MARKER_TO_JOINT_FUNCTION_FACTORY_HH
