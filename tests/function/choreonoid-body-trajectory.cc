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


#define BOOST_TEST_MODULE choreonoid_body_trajectory

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <roboptim/core/visualization/gnuplot.hh>
#include <roboptim/core/visualization/gnuplot-commands.hh>
#include <roboptim/core/visualization/gnuplot-function.hh>

#include "roboptim/retargeting/function/choreonoid-body-trajectory.hh"

#include <cnoid/BodyMotion>

using namespace roboptim;
using namespace roboptim::visualization;
using namespace roboptim::retargeting;

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (simple)
{
  // Loading the motion.
  cnoid::BodyMotionPtr bodyMotion = boost::make_shared<cnoid::BodyMotion> ();

  bodyMotion->loadStandardYAMLformat
    (DATA_DIR "/sample.body-motion.yaml");

  ChoreonoidBodyTrajectory trajectory (bodyMotion, true);

  ChoreonoidBodyTrajectory::discreteInterval_t interval =
    ChoreonoidBodyTrajectory::makeDiscreteInterval
    (ChoreonoidBodyTrajectory::getLowerBound (trajectory.timeRange ()),
     ChoreonoidBodyTrajectory::getUpperBound (trajectory.timeRange ()),
     (ChoreonoidBodyTrajectory::getUpperBound (trajectory.timeRange ())
      - ChoreonoidBodyTrajectory::getLowerBound (trajectory.timeRange ()))
     / 100.
     );

  // Display initial and final trajectory.
  using namespace roboptim::visualization::gnuplot;
  Gnuplot gnuplot = Gnuplot::make_interactive_gnuplot ();
  std::cout
    << (gnuplot
  	<< plot (trajectory, interval)
  	);
}
