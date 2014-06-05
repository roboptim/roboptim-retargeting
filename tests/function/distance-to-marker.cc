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

#include <fstream>

#include <boost/test/unit_test.hpp>

#include <cnoid/BodyLoader>

#include <roboptim/core/indent.hh>
#include <roboptim/core/filter/bind.hh>
#include <roboptim/core/visualization/gnuplot.hh>
#include <roboptim/core/visualization/gnuplot-function.hh>
#include <roboptim/retargeting/morphing.hh>
#include <roboptim/retargeting/function/distance-to-marker.hh>

using namespace roboptim;
using namespace roboptim::retargeting;

typedef JointToMarkerPositionChoreonoid<EigenMatrixDense>
jointToMarker_t;

std::string modelFilePath (HRP4C_YAML_FILE);
std::string morphingFilePath (DATA_DIR "/human-to-hrp4c.morphing.yaml");

#define CHECK_GRADIENT(F, I, X)						\
  BOOST_CHECK_NO_THROW							\
  (									\
   try									\
     {									\
       roboptim::checkGradientAndThrow ((F), (I), (X));			\
     }									\
   catch(const roboptim::BadGradient<EigenMatrixDense>& e)		\
     {									\
       std::cerr << #F << " (" << I << "):\n" << e << std::endl;	\
       throw;								\
     } )

BOOST_AUTO_TEST_CASE (distance_to_marker)
{
  std::ostream& o = std::cout;

  cnoid::BodyLoader loader;

  o << "Loading model " << modelFilePath << iendl;
  cnoid::BodyPtr robotModel = loader.load (modelFilePath);

  // Load morphing data.
  o << "Loading morphing data " << morphingFilePath << iendl;
  roboptim::retargeting::MorphingData morphing =
    roboptim::retargeting::loadMorphingData (morphingFilePath);


  boost::shared_ptr<jointToMarker_t>
    jointToMarker =
    boost::make_shared<jointToMarker_t>
    (robotModel, morphing);

  // Create configuration vector and markers positions vector.
  Function::vector_t x (6 + robotModel->numJoints ());
  Function::vector_t referencePositions
    (static_cast<Function::vector_t::Index> (morphing.markers.size ()) * 3);

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

  // Display function.
  {
    x = Function::vector_t::Random (6 + robotModel->numJoints ());
    referencePositions = (*jointToMarker) (x);
    boost::shared_ptr<roboptim::DifferentiableFunction> f_ =
      boost::make_shared<DistanceToMarker<EigenMatrixDense> >
      (jointToMarker, referencePositions);

    for (std::size_t i = 0;
	 i < static_cast<std::size_t> (f_->inputSize ()); ++i)
      {
      roboptim::Function::vector_t::Index i_ =
	static_cast<roboptim::Function::vector_t::Index> (i);
      roboptim::visualization::Gnuplot gnuplot =
	roboptim::visualization::Gnuplot::make_interactive_gnuplot ();

      roboptim::Function::value_type delta = 2 * M_PI;
      roboptim::Function::discreteInterval_t window
	(x[i_] - delta, x[i_] + delta, 0.1);

      std::vector<boost::optional<roboptim::Function::value_type> >
	values (static_cast<std::size_t> (f_->inputSize ()));
      for (std::size_t j = 0;
	    j < static_cast<std::size_t> (f_->inputSize ()); ++j)
	 if (i != j)
	   values[j] =
	     x[static_cast<roboptim::Function::vector_t::Index> (j)];

       boost::shared_ptr<roboptim::Function> f = roboptim::bind (f_, values);
       gnuplot << roboptim::visualization::gnuplot::set
	 ((boost::format ("arrow from %1%,-100 to %1%,100 nohead lc rgb 'red'")
	   % x[i_]).str ().c_str ())
	       << roboptim::visualization::gnuplot::plot (*f, window);
       std::ofstream file
	 ((boost::format ("/tmp/distance-to-marker-%d.gp") % i).str ().c_str ());
       file << gnuplot;
    }
  }

  // Translate +/- one meter in X, Y, Z
  for (Function::vector_t::Index i = 0; i < 3; ++i)
    {
      x.setZero ();
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

      Function::vector_t markerPosition =
	(*jointToMarker) (x);
      roboptim::retargeting::updateRobotConfiguration
	(robotModel, x);


      o << "Joint to marker" << incindent << iendl
	<<
	"╔════════════════════════╤════════════════════════╤"
	"══════════╤══════════"
	"╤══════════╤══════════╤══════════╤══════════╤══"
	"════════╤══════════╤══════════╗"
	<< iendl
	<< (boost::format ("║ %-22s │ %-22s │ %-8s │ %-8s │ %-8s"
			   " │ %-8s │ %-8s │ %-8s │ %-8s │ %-8s │ %-8s ║")
	    % "Marker Name"
	    % "Attached to body..."
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
	"╠════════════════════════╪════════════════════════╪"
	"══════════╪══════════"
	"╪══════════╪══════════╪══════════╪══════════╪══"
	"════════╪══════════╪══════════╣"
	<< iendl;

      Function::vector_t::Index offset = 0;
      for (std::size_t markerId = 0;
	   offset < markerPosition.size () - 2; ++markerId, offset += 3)
	{
	  std::string markerName = morphing.markers[markerId];
	  std::string bodyName = morphing.attachedBody (markerName);
	  cnoid::Link* link = robotModel->link (bodyName);

	  boost::optional<cnoid::Vector3> markerOffset =
	    morphing.offset (bodyName, markerName);
	  o << (boost::format
		("║ %-22s │ %-22s │ %-8.2d │ %-8.2d │ %-8.2d │ %-8.2d"
		 " │ %-8.2d │ %-8.2d │ %-8.2d │ %-8.2d │ %-8.2d ║")
		% morphing.markers[markerId]
		% (link ? link->name () : "none")
		% markerPosition[offset]
		% markerPosition[offset + 1]
		% markerPosition[offset + 2]
		% (link ? link->position ().translation ()[0] : 0.)
		% (link ? link->position ().translation ()[1] : 0.)
		% (link ? link->position ().translation ()[2] : 0.)
		% (markerOffset ? (*markerOffset)[0] : 0.)
		% (markerOffset ? (*markerOffset)[1] : 0.)
		% (markerOffset ? (*markerOffset)[2] : 0.)
		).str ()
	    << iendl;
	}
      o <<
	"╚════════════════════════╧════════════════════════╧"
	"══════════╧══════════╧══════════"
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

      CHECK_GRADIENT (distance, 0, x);
    }
}
