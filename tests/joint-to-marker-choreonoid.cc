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

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

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
  Function::vector_t result2 = (*jointToMarker) (x);

  BOOST_CHECK (allclose (result, result2));

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

  // Initialize ROS.
  int argc = 1;
  char* argv[] = {"joint-to-marker-choreonoid"};
  ros::init(argc, argv, "roboptim_retargeting");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub =
    n.advertise<visualization_msgs::Marker>("roboptim_retargeting", 1, true);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::SPHERE_LIST;


  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "markers";
  marker.id = 0;
  marker.type = shape;

  // Set the marker action.  Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 0.05x0.05x0.05 here means 5cm radius
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();



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

	// Publish the marker
	marker.points.resize (result.size() / 3);
	for (int i = 0; i < marker.points.size (); ++i)
	  {
	    marker.points[i].x = result[i * 3 + 0];
	    marker.points[i].y = result[i * 3 + 1];
	    marker.points[i].z = result[i * 3 + 2];
	  }
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

  marker_pub.publish(marker);
  r.sleep();
}
