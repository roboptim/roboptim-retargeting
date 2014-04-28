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


#define BOOST_TEST_MODULE minimum_jerk

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <roboptim/core/filter/derivative.hh>
#include <roboptim/core/visualization/gnuplot.hh>
#include <roboptim/core/visualization/gnuplot-commands.hh>
#include <roboptim/core/visualization/gnuplot-function.hh>
#include <roboptim/retargeting/function/minimum-jerk-trajectory.hh>

using namespace roboptim;
using namespace roboptim::visualization;
using namespace roboptim::retargeting;

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (simple)
{
  typedef MinimumJerkTrajectory<EigenMatrixDense>::vector_t vector_t;

  double init = .2;
  double goal = 1.6;

  vector_t x (4);
  x[0] = init;
  x[1] = goal;
  x[2] = x[3] = 0.;

  boost::shared_ptr<MinimumJerkTrajectory<EigenMatrixDense> >
    minimumJerkTrajectory =
    boost::make_shared<MinimumJerkTrajectory<EigenMatrixDense> >
    ();
  minimumJerkTrajectory->setParameters (x);
  MinimumJerkTrajectory<EigenMatrixDense>::discreteInterval_t
    intervalS (0., 1., 0.001);

  using namespace roboptim::visualization::gnuplot;
  Gnuplot gnuplot = Gnuplot::make_interactive_gnuplot ();

  std::cout
    << (gnuplot
	<< set ("multiplot layout 3, 1")
	<< plot (*minimumJerkTrajectory, intervalS)
	<< plot (*derivative (minimumJerkTrajectory, 0), intervalS)
	<< plot (*derivative (derivative (minimumJerkTrajectory, 0), 0), intervalS)
	<< unset ("multiplot")
	);
}
