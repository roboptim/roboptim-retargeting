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

# include <cnoid/BodyIMesh>
# include <cnoid/BodyLoader>
# include <cnoid/BodyMotion>

# include <roboptim/core/problem.hh>
# include <roboptim/core/filter/bind.hh>

# include <roboptim/trajectory/state-function.hh>
# include <roboptim/trajectory/vector-interpolation.hh>

# include <roboptim/retargeting/function/choreonoid-body-trajectory.hh>

# include <roboptim/retargeting/problem/marker-to-joint-function-factory.hh>


namespace roboptim
{
  namespace retargeting
  {
    // Warning: be particularly cautious regarding the loading order
    // as data is inter-dependent.
    void
    buildJointDataFromOptions (MarkerToJointFunctionData& data,
			       const MarkerToJointProblemOptions& options)
    {
      cnoid::BodyLoader loader;

      // data.inputTrajectory = boost::make_shared<cnoid::BodyMotion> ();
      // data.inputTrajectory->loadStandardYAMLformat (options.jointsTrajectory);

      data.robotModel = loader.load (options.robotModel);

      // // Load the trajectory
      // if (options.trajectoryType == "discrete")
      // 	{
      // 	  data.trajectory = boost::make_shared<ChoreonoidBodyTrajectory>
      // 	    (data.jointsTrajectory, true);
      // 	}
      // else
      // 	throw std::runtime_error ("invalid trajectory type");
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
      buildJointDataFromOptions (data, options_);
      MarkerToJointFunctionFactory factory (data);
      data.cost =
	factory.buildFunction<DifferentiableFunction> (options_.cost);
      problem = boost::make_shared<T> (*data.cost);
      //problem->startingPoint () = data.filteredTrajectory->parameters ();
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_PROBLEM_MARKER_TO_JOINT_PROBLEM_BUILDER_HXX
