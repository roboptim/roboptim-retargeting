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
#ifndef ROBOPTIM_RETARGETING_PROBLEM_JOINT_PROBLEM_BUILDER_HXX
# define ROBOPTIM_RETARGETING_PROBLEM_JOINT_PROBLEM_BUILDER_HXX
# include <boost/format.hpp>
# include <boost/make_shared.hpp>

# include <cnoid/BodyLoader>
# include <cnoid/BodyMotion>

# include <roboptim/core/problem.hh>

# include <roboptim/retargeting/function/choreonoid-body-trajectory.hh>

# include <roboptim/retargeting/problem/joint-function-factory.hh>


namespace roboptim
{
  namespace retargeting
  {
    /// \brief Copy a trajectory while excluding some joints.
    ///
    /// \param[in] originalTrajectory original, full, trajectory
    /// \param[in] disabledJoints list of joints to be excluded
    /// \param[in] robot robot model
    boost::shared_ptr<Trajectory<3> >
    filterTrajectory (boost::shared_ptr<Trajectory<3> > originalTrajectory,
		      const std::vector<std::string>& disabledJoints,
		      cnoid::BodyPtr robot)
    {
      std::set<std::string> disabledJointsSet
	(disabledJoints.begin (), disabledJoints.end ());
      if (disabledJoints.size () != disabledJointsSet.size ())
	throw std::runtime_error ("disabled joints are not unique");

      if (disabledJointsSet.size () ==
	  static_cast<std::size_t> (robot->numJoints ()))
	throw std::runtime_error ("all joints are disabled");

      // FIXME: put that in a separate method.
      std::vector<bool> enabledDofs
	(6 + static_cast<std::size_t> (robot->numJoints ()), true);

      std::set<std::string>::const_iterator it;
      for (it = disabledJointsSet.begin (); it != disabledJointsSet.end (); ++it)
	{
	  cnoid::Link* link = robot->link (*it);
	  if (!link)
	    {
	      boost::format fmt ("joint ``%s'' does not exist in robot ``%s''");
	      fmt % *it % robot->modelName ();
	      throw std::runtime_error (fmt.str ());
	    }

	  // add 6 for free-floating and remove 1 as body are labeled
	  // starting from one in Choreonoid
	  int id = 6 + link->index () - 1;
	  if (id < 0)
	    throw std::runtime_error ("conversion from body name"
				      " to index failed (negative index)");
	  enabledDofs[static_cast<std::size_t> (id)] = false;
	  }

      //FIXME: fix and clean this.
      std::size_t nDofsFull = enabledDofs.size ();
      std::size_t nDofsReduced = nDofsFull - disabledJointsSet.size ();

      Trajectory<3>::vector_t::Index reducedTrajectoryParametersSize =
	nDofsReduced * static_cast<Trajectory<3>::vector_t::Index>
	((originalTrajectory->parameters ().size ()) / nDofsFull));
      Trajectory<3>::vector_t reducedTrajectoryParameters (reducedTrajectoryParametersSize);

      Trajectory<3>::vector_t::const_iterator itParam;
      std::size_t idx = 0;
      for (itParam = originalTrajectory->parameters ().begin ();
	   itParam != originalTrajectory->parameters ().end (); ++itParam)
	if (enabledDofs[(itParam - originalTrajectory->parameters ().begin ()) % nDofsFull])
	  reducedTrajectoryParameters[idx] = *itParam;


    }

    void
    buildJointDataFromOptions (JointFunctionData& data,
			       const JointProblemOptions& options)
    {
      cnoid::BodyLoader loader;

      //FIXME: check that data.disabledDofs contains unique elements
      //which are existing joints.

      data.jointsTrajectory = boost::make_shared<cnoid::BodyMotion> ();
      data.jointsTrajectory->loadStandardYAMLformat (options.jointsTrajectory);

      data.robotModel = loader.load (options.robotModel);

      // There is an equivalence between joints and degrees of freedom
      // in Choreonoid due to the fact it only supports one dof
      // joints.
      data.nDofs =
	6 + static_cast<std::size_t> (data.robotModel->numJoints ())
	- options.disabledJoints.size ();


      if (options.trajectoryType == "discrete")
	{
	  data.trajectory = boost::make_shared<ChoreonoidBodyTrajectory>
	    (data.jointsTrajectory, true);
	  data.filteredTrajectory =
	    filterTrajectory
	    (data.trajectory, options.disabledJoints, data.robotModel);
	}
      else
      	throw std::runtime_error ("invalid trajectory type");


    }


    template <typename T>
    JointProblemBuilder<T>::JointProblemBuilder
    (const JointProblemOptions& options)
      : options_ (options)
    {}

    template <typename T>
    JointProblemBuilder<T>::~JointProblemBuilder ()
    {}

    template <typename T>
    std::pair<boost::shared_ptr<T>,
	      boost::shared_ptr<typename T::function_t> >
    JointProblemBuilder<T>::operator () ()
    {
      std::pair<boost::shared_ptr<T>,
		boost::shared_ptr<typename T::function_t > > result;
      JointFunctionData data;
      buildJointDataFromOptions (data, options_);

      JointFunctionFactory factory (data);

      boost::shared_ptr<DifferentiableFunction> cost =
	factory.buildFunction<DifferentiableFunction> (options_.cost);

      boost::shared_ptr<T> problem = boost::make_shared<T> (*cost);

      std::vector<std::string>::const_iterator it;
      for (it = options_.constraints.begin ();
	   it != options_.constraints.end (); ++it)
	{
	  Constraint<DifferentiableFunction> constraint =
	    factory.buildConstraint<DifferentiableFunction> (*it);
	  problem->addConstraint
	    (constraint.function,
	     constraint.intervals,
	     constraint.scales);
	}

      problem->startingPoint () = data.trajectory->parameters ();

      result.first = problem;
      result.second = cost;
      return result;
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_PROBLEM_JOINT_PROBLEM_BUILDER_HXX
