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

#include <fstream>

#include <boost/array.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <roboptim/core/filter/bind.hh>
#include <roboptim/core/visualization/gnuplot.hh>
#include <roboptim/core/visualization/gnuplot-commands.hh>
#include <roboptim/core/visualization/gnuplot-function.hh>

#include "roboptim/retargeting/function/choreonoid-body-trajectory.hh"
#include "roboptim/retargeting/function/body-laplacian-deformation-energy/choreonoid.hh"

#include <cnoid/BodyLoader>
#include <cnoid/BodyMotion>

#include "tests-config.h"

using namespace roboptim;
using namespace roboptim::visualization;
using namespace roboptim::retargeting;

using boost::test_tools::output_test_stream;

//FIXME: we should embed the copy.
std::string modelFilePath
("/home/moulard/HRP4C-release/HRP4Cg2.yaml");
std::string bodyMotionPath
("/home/moulard/29_07-hrp4c-initial-short.yaml");

BOOST_AUTO_TEST_CASE (simple)
{
  // Configure log4cxx
  configureLog4cxx ();

  //FIXME: we should embed the copy.

  // Loading robot.
  cnoid::BodyLoader loader;
  cnoid::BodyPtr robot = loader.load (modelFilePath);
  if (!robot)
    throw std::runtime_error ("failed to load model");

  // Loading the motion.
  cnoid::BodyMotionPtr bodyMotion = boost::make_shared<cnoid::BodyMotion> ();

  //FIXME: we should embed the copy.
  bodyMotion->loadStandardYAMLformat (bodyMotionPath);

  // Body Interaction Mesh
  cnoid::BodyIMeshPtr mesh = boost::make_shared<cnoid::BodyIMesh> ();
  if (!mesh->addBody (robot, bodyMotion))
    throw std::runtime_error ("failed to add body to body interaction mesh");
  if (!mesh->initialize ())
        throw std::runtime_error ("failed to initialize body interaction mesh");

  std::size_t nDofs = 6 + bodyMotion->jointPosSeq ()->numParts ();

  Function::vector_t x (bodyMotion->numFrames () * nDofs);
  x.setZero ();

  boost::shared_ptr<JointToMarkerPositionChoreonoid<
    EigenMatrixDense> >
    jointToMarker =
    boost::make_shared<JointToMarkerPositionChoreonoid<
      EigenMatrixDense> > (mesh);

  boost::shared_ptr<BodyLaplacianDeformationEnergyChoreonoid<
    EigenMatrixDense> >
    cost =
    boost::make_shared<BodyLaplacianDeformationEnergyChoreonoid<
      EigenMatrixDense> >
    (mesh, x, jointToMarker);

  x.setIdentity ();

  std::cout << "Cost Function Display" << '\n';
  std::cout << (*cost) << '\n';

  Function::vector_t markerPositions (mesh->numMarkers () * 3);

  for (std::size_t frameId = 0; frameId < mesh->getNumFrames (); ++frameId)
    {
      markerPositions = (*jointToMarker) (x.segment(frameId * nDofs, nDofs));
      std::cout
	<< "Frame " << frameId << '\n'

	<< "Joint to Marker\n"
	<< markerPositions << '\n'
	<< "Joint to Marker Jacobian\n"
	<< jointToMarker->jacobian (x.segment(frameId * nDofs, nDofs)) << '\n'

	<< "Laplacian Coordinates\n"
	<< (*cost->laplacianCoordinate ()[frameId])
	(markerPositions)
	<< '\n'
	<< "Laplacian Coordinates Jacobian\n"
	<< cost->laplacianCoordinate ()[frameId]->jacobian (markerPositions)
	<< '\n'

	<< "Chain LC\n"
	<< (*cost->chainLc ()[frameId])
	(x.segment(frameId * nDofs, nDofs))
	<< '\n'
	<< "Chain LC Jacobian\n"
	<< cost->chainLc ()[frameId]->jacobian
	(x.segment(frameId * nDofs, nDofs))
	<< '\n'

	<< "LDE\n"
	<< (*cost->lde ()[frameId])
	((*cost->chainLc ()[frameId])
	 (x.segment(frameId * nDofs, nDofs)))
	<< '\n'
        << "LDE Jacobian\n"
	<< (cost->lde ()[frameId])->jacobian
	((*cost->chainLc ()[frameId])
	 (x.segment(frameId * nDofs, nDofs)))
        << '\n'

	<< "Chain\n"
	<< (*cost->chain ()[frameId])
	(x.segment(frameId * nDofs, nDofs))
	<< '\n'
	<< "Chain Jacobian\n"
	<< cost->chain ()[frameId]->jacobian
	(x.segment(frameId * nDofs, nDofs))
	<< '\n'


	<< "\n\n\n";
    }

  std::cout << "Body Laplacian Deformation Energy" << '\n';
  std::cout << (*cost) (x) << '\n';


  std::cout << "Body Laplacian Deformation Energy Jacobian" << '\n';
  std::cout << cost->jacobian (x) << '\n';

  std::ofstream file ("/tmp/body-laplacian-deformation-energy-jac-nodisableddofs.txt");
  file << cost->jacobian (x);
}


/*
  Jacobian can be compared using:

  dwdiff --color --delimiters='[],()' \
  /tmp/body-laplacian-deformation-energy-jac-full.txt \
  /tmp/body-laplacian-deformation-energy-jac-reduced.txt

 */
BOOST_AUTO_TEST_CASE (reduced)
{
  // Configure log4cxx
  configureLog4cxx ();

  //FIXME: we should embed the copy.

  // Loading robot.
  cnoid::BodyLoader loader;
  cnoid::BodyPtr robot = loader.load (modelFilePath);
  if (!robot)
    throw std::runtime_error ("failed to load model");

  // Loading the motion.
  cnoid::BodyMotionPtr bodyMotion = boost::make_shared<cnoid::BodyMotion> ();

  //FIXME: we should embed the copy.
  bodyMotion->loadStandardYAMLformat (bodyMotionPath);

  // Body Interaction Mesh
  cnoid::BodyIMeshPtr mesh = boost::make_shared<cnoid::BodyIMesh> ();
  if (!mesh->addBody (robot, bodyMotion))
    throw std::runtime_error ("failed to add body to body interaction mesh");
  if (!mesh->initialize ())
        throw std::runtime_error ("failed to initialize body interaction mesh");

  std::size_t nFrames = bodyMotion->numFrames ();
  std::size_t oneFrameFullSize =
    6 + bodyMotion->jointPosSeq ()->frame(0).size ();
  std::vector<bool> enabledDofs (oneFrameFullSize, true);

  // Disable useless dofs.
  enabledDofs[6 + 17] = false; // NECK_Y
  enabledDofs[6 + 18] = false; // NECK_R
  enabledDofs[6 + 19] = false; // NECK_P
  enabledDofs[6 + 20] = false; // EYEBROW_P
  enabledDofs[6 + 21] = false; // EYELID_P
  enabledDofs[6 + 22] = false; // EYE_P
  enabledDofs[6 + 23] = false; // EYE_Y
  enabledDofs[6 + 24] = false; // MOUTH_P
  enabledDofs[6 + 25] = false; // LOWERLIP_P
  enabledDofs[6 + 26] = false; // UPPERLIP_P
  enabledDofs[6 + 27] = false; // CHEEK_P
  enabledDofs[6 + 34] = false; // R_HAND_J0
  enabledDofs[6 + 35] = false; // R_HAND_J1
  enabledDofs[6 + 42] = false; // L_HAND_J0
  enabledDofs[6 + 43] = false; // L_HAND_J1

  std::size_t nEnabledDofs =
    std::count (enabledDofs.begin (), enabledDofs.end (), true);

  // X vector (full form)
  Function::vector_t x (nFrames * oneFrameFullSize);
  x.setZero ();

  // X vector (reduced form)
  Function::vector_t xReduced (nFrames * nEnabledDofs);
  xReduced.setZero ();


  typedef roboptim::Function::value_type value_type;
  std::vector<boost::optional<value_type> > boundDofs
    (oneFrameFullSize);
  for (std::size_t jointId = 0; jointId < oneFrameFullSize; ++jointId)
    if (!enabledDofs[jointId])
      boundDofs[jointId] = x[jointId];

  std::vector<boost::optional<value_type> > boundDofsAllFrames
    (nFrames * oneFrameFullSize);
  for (std::size_t frame = 0; frame < nFrames; ++frame)
    for (std::size_t jointId = 0;
	 jointId < oneFrameFullSize; ++jointId)
      if (!enabledDofs[jointId])
	boundDofsAllFrames[frame * oneFrameFullSize + jointId]
	  = x[jointId];

  boost::shared_ptr<JointToMarkerPositionChoreonoid<
    EigenMatrixDense> >
    jointToMarker =
    boost::make_shared<JointToMarkerPositionChoreonoid<
      EigenMatrixDense> > (mesh);

  boost::shared_ptr<BodyLaplacianDeformationEnergyChoreonoid<
    EigenMatrixDense> >
    cost =
    boost::make_shared<BodyLaplacianDeformationEnergyChoreonoid<
      EigenMatrixDense> > (mesh, x, jointToMarker);

  boost::shared_ptr<DifferentiableFunction>
    costFiltered = bind<DifferentiableFunction> (cost, boundDofsAllFrames);

  std::cout << "X (input)\n";
  std::cout << x << '\n';

  std::cout << "X (input, reduced)\n";
  std::cout << xReduced << '\n';

  std::cout << "Body Laplacian Deformation Energy\n";
  std::cout << (*costFiltered) (xReduced) << '\n';

  std::cout << "Cost Function Display\n";
  std::cout << (*costFiltered) << '\n';

  std::cout << "Body Laplacian Deformation Energy Jacobian (full)\n";
  std::cout << cost->jacobian (x) << '\n';

  std::cout << "Body Laplacian Deformation Energy Jacobian (bound)\n";
  std::cout << costFiltered->jacobian (xReduced) << '\n';

  std::ofstream file ("/tmp/body-laplacian-deformation-energy-jac-full.txt");
  file << cost->jacobian (x);
  std::ofstream file2 ("/tmp/body-laplacian-deformation-energy-jac-reduced.txt");
  file2 << costFiltered->jacobian (xReduced);
}
