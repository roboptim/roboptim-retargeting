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
#ifndef ROBOPTIM_RETARGETING_PROBLEM_MARKER_TO_JOINT_PROBLEM_BUILDER_HXX
# define ROBOPTIM_RETARGETING_PROBLEM_MARKER_TO_JOINT_PROBLEM_BUILDER_HXX
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

# include <roboptim/retargeting/function/choreonoid-body-trajectory.hh>
# include <roboptim/retargeting/function/libmocap-marker-trajectory.hh>
# include <roboptim/retargeting/problem/marker-to-joint-function-factory.hh>

// for trajectory filtering
# include <roboptim/retargeting/problem/joint-problem-builder.hh>


namespace roboptim
{
  namespace retargeting
  {
    // Warning: be particularly cautious regarding the loading order
    // as data is inter-dependent.
    void
    buildMarkerToJointDataFromOptions
    (MarkerToJointFunctionData& data,
     const MarkerToJointProblemOptions& options)
    {
      data.frameId = options.frameId;

      if (data.initialized)
	return;
      data.initialized = true;

      cnoid::BodyLoader loader;

      data.markerSet =
	libmocap::MarkerSetFactory ().load (options.markerSet);
      data.markersTrajectory =
	libmocap::MarkerTrajectoryFactory ().load (options.markersTrajectory);
      data.robotModel = loader.load (options.robotModel);

      if (!data.inputTrajectory)
	{
	  if (options.trajectoryType == "discrete")
	    {
	      // load the trajectory
	      boost::shared_ptr<LibmocapMarkerTrajectory> trajectory =
		boost::make_shared<LibmocapMarkerTrajectory> (data.markersTrajectory);

	      // trim the trajectory to satisfy the start frame / length arguments.
	      data.inputTrajectory = safeGet (trajectory).trim
		(static_cast<Function::size_type> (options.startFrame),
		 static_cast<Function::size_type> (options.length));
	    }
	  else
	    throw std::runtime_error ("invalid trajectory type");
	}

      Function::size_type nFrames =
	static_cast<Function::size_type> (data.markersTrajectory.numFrames ());
      Function::value_type dt =
	static_cast<Function::value_type>
	(1. / data.markersTrajectory.dataRate ());

      Function::size_type nDofsFull =
	static_cast<Function::size_type> (6 + data.robotModel->numJoints ());
      //FIXME: unsafe if vector is ill-formed
      Function::size_type nDofsReduced = nDofsFull -
	static_cast<Function::size_type>
	(data.disabledJointsConfiguration.size ());

      if (!data.outputTrajectoryReduced)
	data.outputTrajectoryReduced =
	  boost::make_shared<VectorInterpolation>
	  (Function::vector_t (nDofsReduced * nFrames),
	   nDofsReduced, dt);
      if (!data.outputTrajectory)
	{
	  Function::vector_t parameters (nDofsFull * nFrames);
	  parameters.setZero ();

	  const cnoid::Listing& pose =
	    *data.robotModel->info ()->findListing ("standardPose");
	  for (int i = 0; i < nDofsFull - 6; ++i)
	    {
	      std::stringstream stream;
	      stream << pose.at (i)->toString ();
	      double value;
	      stream >> value;

	      // convert degree into radian here
	      parameters[i + 6] = value * (M_PI / 180.);
	    }

	  data.outputTrajectory =
	    boost::make_shared<VectorInterpolation>
	    (parameters, nDofsReduced, dt);
	}

      data.disabledJointsConfiguration =
	disabledJointsConfiguration
	(options.disabledJoints, data.outputTrajectory, data.robotModel);
    }


    template <typename T>
    MarkerToJointProblemBuilder<T>::MarkerToJointProblemBuilder
    (const MarkerToJointProblemOptions& options)
      : options_ (options)
    {}

    template <typename T>
    MarkerToJointProblemBuilder<T>::~MarkerToJointProblemBuilder ()
    {}

    template <typename T>
    void
    MarkerToJointProblemBuilder<T>::operator ()
      (boost::shared_ptr<T>& problem, MarkerToJointFunctionData& data)
    {
      buildMarkerToJointDataFromOptions (data, options_);
      MarkerToJointFunctionFactory factory (data);
      data.cost =
	factory.buildFunction<DifferentiableFunction> (options_.cost);
      problem = boost::make_shared<T> (*data.cost);


      std::vector<std::string>::const_iterator it;
      for (it = options_.constraints.begin ();
	   it != options_.constraints.end (); ++it)
	{
	  Constraint<DifferentiableFunction> constraint =
	    factory.buildConstraint<DifferentiableFunction> (*it);

	  boost::shared_ptr<LinearFunction> linearConstraint =
	    boost::dynamic_pointer_cast<LinearFunction>
	    (constraint.function);
	  if (linearConstraint)
	    problem->template addConstraint<LinearFunction>
	      (linearConstraint,
	       constraint.intervals,
	       constraint.scales);
	  else
	    problem->addConstraint
	      (constraint.function,
	       constraint.intervals,
	       constraint.scales);
	}

      Function::vector_t::Index length = data.nDofsFiltered ();

      // for first frame, start from half-sitting
      if (options_.frameId == 0)
	{
	  // FIXME: this "reference pose" should be loaded and filtered correctly.
	  const cnoid::Listing& pose =
	    *data.robotModel->info ()->findListing ("standardPose");
	  Function::vector_t referencePose (length);
	  referencePose.template segment<6> (0).setZero ();
	  for (int i = 0; i < length - 6; ++i)
	    {
	      std::stringstream stream;
	      stream << pose.at (i)->toString ();
	      double value;
	      stream >> value;

	      // convert degree into radian here
	      referencePose[i + 6] = value * (M_PI / 180.);
	    }

	  problem->startingPoint () = referencePose;
	}
      // for other frames, start from previous frame
      else
	{
	  Function::vector_t::Index start =
	    static_cast<Function::vector_t::Index>
	    ((options_.frameId - 1) * length);

	  problem->startingPoint () =
	    data.outputTrajectoryReduced->parameters ().segment
	    (start, length);
	}

    }
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_PROBLEM_MARKER_TO_JOINT_PROBLEM_BUILDER_HXX
