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
# include <boost/make_shared.hpp>

# include <roboptim/core/problem.hh>

# include <roboptim/trajectory/vector-interpolation.hh>

# include <roboptim/retargeting/problem/joint-function-factory.hh>


namespace roboptim
{
  namespace retargeting
  {
    void
    buildJointDataFromOptions (JointFunctionData&,
			       const JointProblemOptions&)
    {
      // data.jointsTrajectory =
      // 	libmocap::JointTrajectoryFactory ().load (options.jointsTrajectory);

      // if (options.trajectoryType == "discrete")
      // 	data.trajectory =
      // 	  convertToTrajectory<VectorInterpolation> (data.jointsTrajectory);
      // else
      // 	throw std::runtime_error ("invalid trajectory type");
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
