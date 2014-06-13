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
# include <algorithm>

# include <boost/format.hpp>
# include <boost/make_shared.hpp>
# include <boost/optional.hpp>

# include <cnoid/BodyLoader>
# include <cnoid/BodyMotion>

# include <roboptim/core/problem.hh>
# include <roboptim/core/filter/bind.hh>

# include <roboptim/trajectory/state-function.hh>
# include <roboptim/trajectory/vector-interpolation.hh>

# include <roboptim/retargeting/morphing.hh>
# include <roboptim/retargeting/function/choreonoid-body-trajectory.hh>

# include <roboptim/retargeting/problem/joint-function-factory.hh>
# include <roboptim/retargeting/utility.hh>


namespace roboptim
{
  namespace retargeting
  {
    /// \brief Store which joints are disabled and what value they
    ///        should be set in this case.
    ///
    /// By default, all joints are enabled.
    ///
    /// The user can also pass a set of joints to be filtered out before
    /// starting the optimization process. The joints are described by
    /// their name.
    ///
    /// Invalid joint names or duplicated joint are illegal.
    ///
    /// This function build the set of enabled joints from this vector
    /// of strings. This is represented by a vector of optional
    /// values. The index is the joint id (0...6 is free-floating then
    /// the joints configurations). The vector value is either empty
    /// (the joint value will be determined by the optimization
    /// process) or defined to a constant. In the latter case, the
    /// joint is excluded from the optimization process and will
    /// always be equal to this value. The constant (default) value of
    /// the joint is the value of its joint in the first frame of the
    /// joint trajectory.
    ///
    /// \param[in] disabledJoints list of joints to be excluded from the
    ///            optimization process and identified by their name
    /// \param[in] originalTrajectory full joint trajectory
    /// \param[in] robotModel robot model loaded by Choreonoid and
    ///            providing the joint name to index mapping
    /// \return vector of enabled and disabled joints (no value means that
    ///         joint is disabled, i.e. not to be taken into account during the
    ///         optimization process and a value means this joint will be excluded
    ///         from the optimization process and always take this value).
    std::vector<boost::optional<Function::value_type> >
    disabledJointsConfiguration
    (const std::vector<std::string>& disabledJoints,
     TrajectoryShPtr originalTrajectory,
     cnoid::BodyPtr robotModel)
    {
      std::set<std::string> disabledJointsSet
	(disabledJoints.begin (), disabledJoints.end ());
      if (disabledJoints.size () != disabledJointsSet.size ())
	throw std::runtime_error ("disabled joints are not unique");

      if (disabledJointsSet.size () ==
	  static_cast<std::size_t> (robotModel->numJoints ()))
	throw std::runtime_error ("all joints are disabled");

      std::vector<boost::optional<Function::value_type> > configuration
	(6 + static_cast<std::size_t> (robotModel->numJoints ()),
	 boost::none_t ());

      std::set<std::string>::const_iterator it;
      for (it = disabledJointsSet.begin (); it != disabledJointsSet.end (); ++it)
	{
	  cnoid::Link* link = robotModel->link (*it);
	  if (!link)
	    {
	      boost::format fmt ("joint ``%s'' does not exist in robotModel ``%s''");
	      fmt % *it % robotModel->modelName ();
	      throw std::runtime_error (fmt.str ());
	    }

	  // add 6 for free-floating and remove 1 as body are labeled
	  // starting from one in Choreonoid
	  int id = 6 + link->index () - 1;
	  if (id < 0)
	    throw std::runtime_error ("conversion from body name"
				      " to index failed (negative index)");
	  configuration[static_cast<std::size_t> (id)] =
	    originalTrajectory->parameters ()[id];
	}
      return configuration;
    }

    /// \brief Copy a trajectory while excluding some joints.
    ///
    /// \param[in] originalTrajectory original, full, trajectory
    /// \param[in] disabledJointsConfiguration list of joints to be excluded
    /// \return filtered trajectory
    TrajectoryShPtr
    filterTrajectory (TrajectoryShPtr originalTrajectory,
		      const std::vector<boost::optional<Function::value_type> >&
		      disabledJointsConfiguration)
    {
      typedef Trajectory::vector_t::Index index_t;

      index_t nDofsFull =
	static_cast<index_t> (disabledJointsConfiguration.size ());

      index_t nDofsReduced =
	std::count (disabledJointsConfiguration.begin (),
		    disabledJointsConfiguration.end (),
		    boost::none_t ());
      if (nDofsReduced < 1)
	  throw std::runtime_error ("all DOFs have been disabled");
      index_t nFrames =
	static_cast<index_t> (originalTrajectory->parameters ().size ())
	/ nDofsFull;
      index_t reducedTrajectoryParametersSize = nDofsReduced * nFrames;

      Trajectory::vector_t
	reducedTrajectoryParameters (reducedTrajectoryParametersSize);
      reducedTrajectoryParameters.setZero ();

      index_t idx = 0;
      index_t idxOriginal = 0;
      while (idx < reducedTrajectoryParametersSize &&
	     idxOriginal < originalTrajectory->parameters ().size ())
	{
	  if (!disabledJointsConfiguration[static_cast<std::size_t> (idxOriginal % nDofsFull)])
	    reducedTrajectoryParameters[idx++] = originalTrajectory->parameters ()[idxOriginal];
	  idxOriginal++;
	}

      TrajectoryShPtr reducedTrajectory =
	boost::make_shared<VectorInterpolation>
	(reducedTrajectoryParameters,
	 nDofsReduced, static_cast<index_t> (originalTrajectory->length ()) / nDofsFull);

      return reducedTrajectory;
    }

    // Warning: be particularly cautious regarding the loading order
    // as data is inter-dependent.
    void
    buildJointDataFromOptions (JointFunctionData& data,
			       const JointProblemOptions& options)
    {
      cnoid::BodyLoader loader;

      data.jointsTrajectory = boost::make_shared<cnoid::BodyMotion> ();
      data.jointsTrajectory->loadStandardYAMLformat (options.jointsTrajectory);

      data.robotModel = loader.load (options.robotModel);
      data.morphing = loadMorphingData (options.morphing);
      data.markerMapping = buildMarkerMappingFromMorphing (data.morphing);

      // Load the trajectory
      if (options.trajectoryType == "discrete")
	{
	  // load the trajectory
	  boost::shared_ptr<ChoreonoidBodyTrajectory> trajectory =
	    boost::make_shared<ChoreonoidBodyTrajectory>
	    (data.jointsTrajectory, true);

	  // trim it before insertion to satisfy startFrame / length
	  // options.
	  data.trajectory = safeGet (trajectory).trim
	    (static_cast<Function::size_type> (options.startFrame),
	     static_cast<Function::size_type> (options.length));
	}
      else
      	throw std::runtime_error ("invalid trajectory type");

      // Create the interaction mesh
      data.interactionMesh =
	buildInteractionMeshFromMarkerMotion
	(data.trajectory, data.markerMapping);
      if (!data.interactionMesh)
	throw std::runtime_error ("failed to build the interaction mesh");

      // Filter the trajectory (do not re-order as the initial trajectory is needed!)

      data.disabledJointsConfiguration =
	disabledJointsConfiguration
	(options.disabledJoints, data.trajectory, data.robotModel);

      for (Function::vector_t::Index frame = 0; frame < data.nFrames (); ++frame)
	data.disabledJointsTrajectory.insert
	  (data.disabledJointsTrajectory.end (),
	   data.disabledJointsConfiguration.begin (),
	   data.disabledJointsConfiguration.end ());

      data.filteredTrajectory =
	filterTrajectory
	(data.trajectory, data.disabledJointsConfiguration);
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
    void
    JointProblemBuilder<T>::operator () (boost::shared_ptr<T>& problem,
					 JointFunctionData& data)
    {
      buildJointDataFromOptions (data, options_);

      std::size_t nConstraints =
	static_cast<std::size_t> (data.nFrames ()) - 2;


      JointFunctionFactory factory (data);

      data.cost =
	factory.buildFunction<DifferentiableFunction> (options_.cost);

      problem = boost::make_shared<T> (*data.cost);

      std::vector<std::string>::const_iterator it;
      for (it = options_.constraints.begin ();
	   it != options_.constraints.end (); ++it)
	{
	  Constraint<DifferentiableFunction> constraint =
	    factory.buildConstraint<DifferentiableFunction> (*it);

	  switch (constraint.type)
	    {
	    case Constraint<T>::CONSTRAINT_TYPE_ONCE:
	      {
		problem->addConstraint
		  (constraint.function,
		   constraint.intervals,
		   constraint.scales);
		break;
	      }
	    case Constraint<T>::CONSTRAINT_TYPE_PER_FRAME:
	      {
		for (std::size_t i = 0; i < nConstraints; ++i)
		  {
		    const Function::value_type t =
		      (static_cast<Function::value_type> (i) + 1.) /
		      (static_cast<Function::value_type> (nConstraints) + 1.);
		    ROBOPTIM_RETARGETING_ASSERT (t > 0. && t < 1.);

		    boost::shared_ptr<DifferentiableFunction> f
		      (new roboptim::StateFunction<Trajectory >
		       (*data.filteredTrajectory,
			constraint.function,
			t * tMax,
			static_cast<Function::size_type>
			(constraint.stateFunctionOrder)));

		    f = bind (f, data.disabledJointsTrajectory);
		    problem->addConstraint
		      (f, constraint.intervals, constraint.scales);
		  }
		break;
	      }
	    default:
	      ROBOPTIM_RETARGETING_ASSERT (0 && "should never happen");
	    }
	}

      problem->startingPoint () = data.filteredTrajectory->parameters ();
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_PROBLEM_JOINT_PROBLEM_BUILDER_HXX
