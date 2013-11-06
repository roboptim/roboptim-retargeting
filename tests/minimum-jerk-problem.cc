// Copyright (C) 2013 by Thomas Moulard, AIST, CNRS, INRIA.
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


#define BOOST_TEST_MODULE minimum_jerk_problem

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <roboptim/core/visualization/gnuplot.hh>
#include <roboptim/core/visualization/gnuplot-commands.hh>
#include <roboptim/core/visualization/gnuplot-function.hh>

#include "roboptim/retargeting/problem/minimum-jerk.hh"

#include <cnoid/BodyLoader>

#include "tests-config.h"

using namespace roboptim;
using namespace roboptim::visualization;
using namespace roboptim::retargeting;

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (simple)
{
  // Configure log4cxx
  configureLog4cxx ();

  //FIXME: we should embed the copy.
  std::string modelFilePath
    ("/home/moulard/HRP4C-release/HRP4Cg2.yaml");

  // Loading robot.
  cnoid::BodyLoader loader;
  cnoid::BodyPtr robot = loader.load (modelFilePath);
  if (!robot)
    throw std::runtime_error ("failed to load model");

  bool enableFreeze = true;
  bool enableVelocity = true;
  bool enablePosition = true;
  bool enableCollision = false;
  bool enableTorque = false;
  bool enableZmp = true;
  std::string solverName = "cfsqp";
  std::vector<bool> enabledDofs (6 + 44, true);

  typedef problem::MinimumJerk::size_type size_type;
  typedef problem::MinimumJerk::solver_t solver_t;

  problem::MinimumJerkShPtr_t minimumJerkProblem =
    roboptim::retargeting::problem::MinimumJerk::
    buildVectorInterpolationBasedOptimizationProblem
    (robot, 10, 0.1, enableFreeze, enableVelocity,
     enablePosition, enableCollision,
     enableTorque, enableZmp, solverName, enabledDofs);
  minimumJerkProblem->solve ();

  // Rebuild final trajectory.
  if (minimumJerkProblem->result ().which () == solver_t::SOLVER_ERROR)
    {
      std::cerr << "error" << std::endl;
      roboptim::SolverError error =
  	boost::get<roboptim::SolverError> (minimumJerkProblem->result ());
      std::cerr << "Result:\n" << error << std::endl;
      return;
    }

  boost::shared_ptr<VectorInterpolation >
    finalTrajectoryFct;

  if (minimumJerkProblem->result ().which () == solver_t::SOLVER_VALUE_WARNINGS)
    {
      std::cerr << "warnings" << std::endl;
      roboptim::ResultWithWarnings result =
  	boost::get<roboptim::ResultWithWarnings> (minimumJerkProblem->result ());
      std::cerr << "Result:\n" << result << std::endl;
      finalTrajectoryFct =
	vectorInterpolation
	(result.x, static_cast<size_type> (minimumJerkProblem->nDofs ()),
	 minimumJerkProblem->dt ());
    }

  if (minimumJerkProblem->result ().which () == solver_t::SOLVER_VALUE)
    {
      std::cerr << "ok" << std::endl;
      roboptim::Result result =
	boost::get<roboptim::Result> (minimumJerkProblem->result ());
      std::cerr << "Result:\n" << result << std::endl;
      finalTrajectoryFct =
	vectorInterpolation
	(result.x, static_cast<size_type> (minimumJerkProblem->nDofs ()),
	 minimumJerkProblem->dt ());
    }

  assert (!!finalTrajectoryFct);

  // Display initial and final trajectory.
  using namespace roboptim::visualization::gnuplot;
  Gnuplot gnuplot = Gnuplot::make_interactive_gnuplot ();
  std::cout
    << (gnuplot
  	<< plot (*finalTrajectoryFct, minimumJerkProblem->interval ())
  	);
}
