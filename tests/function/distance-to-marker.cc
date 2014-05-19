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

#define BOOST_TEST_MODULE distance_to_marker

#include <boost/test/unit_test.hpp>

#include <cnoid/BodyIMesh>
#include <cnoid/BodyLoader>
#include <cnoid/BodyMotion>

#include <roboptim/core/indent.hh>
#include <roboptim/retargeting/function/distance-to-marker.hh>

using namespace roboptim;
using namespace roboptim::retargeting;

typedef JointToMarkerPositionChoreonoid<EigenMatrixDense>
jointToMarker_t;

std::string modelFilePath (HRP4C_YAML_FILE);


BOOST_AUTO_TEST_CASE (distance_to_marker)
{
  std::ostream& o = std::cout;

  cnoid::BodyLoader loader;

  o << "Loading model " << modelFilePath << iendl;
  cnoid::BodyPtr robotModel = loader.load (modelFilePath);

  // Create the interaction mesh
  cnoid::BodyIMeshPtr interactionMesh =
    boost::make_shared<cnoid::BodyIMesh> ();

  if (!interactionMesh->addBody
      (robotModel, boost::make_shared<cnoid::BodyMotion> ()))
    throw std::runtime_error ("failed to add body to body interaction mesh");
  // if (!interactionMesh->initialize ())
  //   throw std::runtime_error ("failed to initialize body interaction mesh");
  interactionMesh->initialize ();


  boost::shared_ptr<jointToMarker_t>
    jointToMarker =
    boost::make_shared<jointToMarker_t>
    (interactionMesh);

  // Create configuration vector and markers positions vector.
  Function::vector_t x (6 + robotModel->numJoints ());
  Function::vector_t referencePositions (interactionMesh->numMarkers () * 3);

  // Make sure that cost is zero if marker position and reference
  // position match.
  for (std::size_t i = 0; i < 10; ++i)
    {
      x = Function::vector_t::Random (6 + robotModel->numJoints ());
      referencePositions = (*jointToMarker) (x);
      DistanceToMarker<EigenMatrixDense> distance
	(jointToMarker, referencePositions);
      BOOST_CHECK_CLOSE (distance(x)[0], 0., 1e-6);
    }

  // Set configuration to zero and use a reference.
  x.setZero ();
  referencePositions = (*jointToMarker) (x);
  DistanceToMarker<EigenMatrixDense> distance
    (jointToMarker, referencePositions);

  // Translate +/- one meter in X, Y, Z
  for (Function::vector_t::Index i = 0; i < 3; ++i)
    {
      x[i] = 1.;
      // the result is 1/2 (1/2 * \sum 1^2)
      BOOST_CHECK_CLOSE (distance(x)[0], 0.5, 1e-6);
      x[i] = -1.;
      // the result is 1/2 (1/2 * \sum 1^2)
      BOOST_CHECK_CLOSE (distance(x)[0], 0.5, 1e-6);
    }


  for (std::size_t i = 0; i < 10; ++i)
    {
      o << "➔ sampling configuration" << iendl;

      x = Function::vector_t::Random (6 + robotModel->numJoints ());

      o << "Configuration" << incindent << iendl
	<< "╔════════════════════════╤══════════╗"
	<< iendl
	<< "║ DOF name               │ Value    ║"
	<< iendl
	<< "╠════════════════════════╪══════════╣"
	<< iendl ;

      for (DifferentiableFunction::jacobian_t::Index jointId = 0;
	   jointId < x.size (); ++jointId)
	{
	  std::string jointName;

	  switch (jointId)
	    {
	    case 0:
	      jointName = "TX";
	      break;
	    case 1:
	      jointName = "TY";
	      break;
	    case 2:
	      jointName = "TZ";
	      break;
	    case 3:
	      jointName = "RX";
	      break;
	    case 4:
	      jointName = "RY";
	      break;
	    case 5:
	      jointName = "RZ";
	      break;
	    default:
	      jointName = robotModel->joint
		(static_cast<int> (jointId - 6))->name ();
	    }
	  o << (boost::format ("║ %-22s │ %-8.2d ║")
		% jointName % x[jointId]).str () << iendl;
	}

      o << "╚════════════════════════╧══════════╝"
	<< decindent << iendl << iendl;

      Function::vector_t configuration =
	(*jointToMarker) (x);
      roboptim::retargeting::updateRobotConfiguration
	(robotModel, configuration);


      o << "Joint to marker" << incindent << iendl
	<<
	"╔════════════════════════╤══════════╤══════════"
	"╤══════════╤══════════╤══════════╤══════════╤══"
	"════════╤══════════╤══════════╗"
	<< iendl
	<< (boost::format ("║ %-22s │ %-8s │ %-8s │ %-8s"
			   " │ %-8s │ %-8s │ %-8s │ %-8s │ %-8s │ %-8s ║")
	    % "Marker attached to..."
	    % "Marker X"
	    % "Marker Y"
	    % "Marker Z"
	    % "Body X"
	    % "Body Y"
	    % "Body Z"
	    % "Offset X"
	    % "Offset Y"
	    % "Offset Z").str ()
	<< iendl
	<<
	"╠════════════════════════╪══════════╪══════════"
	"╪══════════╪══════════╪══════════╪══════════╪══"
	"════════╪══════════╪══════════╣"
	<< iendl;

      Function::vector_t::Index offset = 0;
      for (std::size_t markerId = 0;
	   offset < configuration.size () - 2; ++markerId, offset += 3)
	{
	  cnoid::Link* link = interactionMesh->bodyInfo (0).markers[markerId]->link;
	  boost::optional<cnoid::Vector3> markerOffset =
	    interactionMesh->bodyInfo (0).markers[markerId]->localPos;
	  o << (boost::format
		("║ %-22s │ %-8.2d │ %-8.2d │ %-8.2d │ %-8.2d"
		 " │ %-8.2d │ %-8.2d │ %-8.2d │ %-8.2d │ %-8.2d ║")
		% (link ? link->name () : "none")
		% configuration[offset]
		% configuration[offset + 1]
		% configuration[offset + 2]
		% (link ? link->position ().translation ()[0] : 0.)
		% (link ? link->position ().translation ()[1] : 0.)
		% (link ? link->position ().translation ()[2] : 0.)
		% (markerOffset ? (*markerOffset)[0] : 0.)
		% (markerOffset ? (*markerOffset)[1] : 0.)
		% (markerOffset ? (*markerOffset)[2] : 0.)
		).str ()
	    << iendl;
	}
      o << "╚════════════════════════╧══════════╧══════════╧══════════"
	"╧══════════╧══════════╧══════════╧══════════╧══════════╧══════════╝"
	<< decindent << iendl << iendl;

      DifferentiableFunction::jacobian_t J = distance.jacobian (x);

      o << "Distance to Marker Jacobian" << incindent << iendl
	<< "╔════════════════════════╤══════════╗"
	<< iendl
	<< "║ DOF name               │ Value    ║"
	<< iendl
	<< "╠════════════════════════╪══════════╣"
	<< iendl ;

      for (DifferentiableFunction::jacobian_t::Index jointId = 0;
	   jointId < J.cols (); ++jointId)
	{
	  std::string jointName;

	  switch (jointId)
	    {
	    case 0:
	      jointName = "TX";
	      break;
	    case 1:
	      jointName = "TY";
	      break;
	    case 2:
	      jointName = "TZ";
	      break;
	    case 3:
	      jointName = "RX";
	      break;
	    case 4:
	      jointName = "RY";
	      break;
	    case 5:
	      jointName = "RZ";
	      break;
	    default:
	      jointName = robotModel->joint
		(static_cast<int> (jointId - 6))->name ();
	    }


	  o << (boost::format ("║ %-22s │ %-8.2d ║")
		% jointName % J (0, jointId)).str () << iendl;
	}

      o << "╚════════════════════════╧══════════╝"
	<< decindent << iendl << iendl
	<< "Distance to marker value: " << distance (x)[0] << iendl
	<< "Average Absolute Deviation: " << 2 * std::sqrt (distance (x)[0]) << iendl
	<< iendl << iendl
	;
    }
}
