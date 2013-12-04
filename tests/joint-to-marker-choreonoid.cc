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

#include <roboptim/core/indent.hh>
#include <roboptim/core/visualization/gnuplot.hh>
#include <roboptim/core/visualization/gnuplot-commands.hh>
#include <roboptim/core/visualization/gnuplot-function.hh>

#include "roboptim/retargeting/function/joint-to-marker/choreonoid.hh"

#include <cnoid/BodyLoader>
#include <cnoid/BodyMotion>

#include "tests-config.h"

using namespace roboptim;
using namespace roboptim::visualization;
using namespace roboptim::retargeting;

using boost::test_tools::output_test_stream;

//FIXME: we should embed the copies.
std::string modelFilePath
("/home/moulard/HRP4C-release/HRP4Cg2.yaml");
std::string bodyMotionPath
("/home/moulard/29_07-hrp4c-initial-short.yaml");


typedef
JointToMarkerPositionChoreonoid<EigenMatrixDense> JointToMarker_t;
typedef
boost::shared_ptr<JointToMarker_t> JointToMarkerShPtr_t;

Function::vector_t evaluateAndPrintTestResult
(std::ostream& o, JointToMarkerShPtr_t jointToMarker, Function::vector_t& x)
{
  jointToMarker->shouldUpdate ();
  Function::vector_t result = (*jointToMarker) (x);
  o << "Evaluating joint-to-marker function" << iendl
    << "X:" << incindent << iendl
    << x << decindent << iendl
    << "Result:" << incindent << iendl
    << result << decindent << iendl;
  return result;
}

BOOST_AUTO_TEST_CASE (simple)
{
  // Configure log4cxx
  configureLog4cxx ();

  // Loading robot.
  cnoid::BodyLoader loader;
  cnoid::BodyPtr robot = loader.load (modelFilePath);
  if (!robot)
    throw std::runtime_error ("failed to load model");

  // Loading the motion.
  cnoid::BodyMotionPtr bodyMotion = boost::make_shared<cnoid::BodyMotion> ();
  bodyMotion->loadStandardYAMLformat (bodyMotionPath);

  // Body Interaction Mesh
  cnoid::BodyIMeshPtr mesh = boost::make_shared<cnoid::BodyIMesh> ();
  if (!mesh->addBody (robot, bodyMotion))
    throw std::runtime_error ("failed to add body to body interaction mesh");
  if (!mesh->initialize ())
        throw std::runtime_error ("failed to initialize body interaction mesh");

  Function::vector_t x
    (6 + mesh->bodyInfo (0).body->numJoints ());
  Function::result_t result;
  Function::vector_t previousResult;
  JointToMarkerShPtr_t jointToMarker =
    boost::make_shared<JointToMarker_t> (mesh, 0);

  // Printing object
  {
    std::cout << (*jointToMarker) << iendl;
    std::cout << iendl;
  }

  for (int frameId = 0; frameId < bodyMotion->numFrames (); ++frameId)
    {
      std::cout << "Frame: " << frameId << iendl;

      jointToMarker->frameId () = frameId;

      // x = 0
      {
	x.setZero ();
	result = evaluateAndPrintTestResult (std::cout, jointToMarker, x);
	std::cout
	  << "Gradient:" << incindent << iendl
	  << jointToMarker->gradient (x) << decindent << iendl
	  << iendl;
	std::cout << iendl;
      }

      // translate 1 meter front in X, Y and Z directions to check
      // for translation handling.
      previousResult = result;
      for (int i = 0; i < 3; ++ i)
	{
	  x.setZero ();
	  x[i] = 1.;
	  result = evaluateAndPrintTestResult (std::cout, jointToMarker, x);
	  std::cout << iendl;

	  for (int idx = 0; idx < result.size (); ++idx)
	    if (idx % 3 == i)
	      BOOST_CHECK_CLOSE (result[idx] - previousResult[idx], 1., 1e-6);
	    else
	      BOOST_CHECK_CLOSE (result[idx] - previousResult[idx], 0., 1e-6);
	}
    }
}
