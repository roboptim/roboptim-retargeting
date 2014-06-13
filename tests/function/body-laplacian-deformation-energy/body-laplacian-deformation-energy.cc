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

using namespace roboptim;
using namespace roboptim::visualization;
using namespace roboptim::retargeting;

using boost::test_tools::output_test_stream;

std::string modelFilePath (HRP4C_YAML_FILE);

BOOST_AUTO_TEST_CASE (simple)
{
  // Loading robot.
  cnoid::BodyLoader loader;
  cnoid::BodyPtr robot = loader.load (modelFilePath);
  if (!robot)
    throw std::runtime_error ("failed to load model");
  // Loading the motion.
  cnoid::BodyMotionPtr bodyMotion = boost::make_shared<cnoid::BodyMotion> ();

  bodyMotion->loadStandardYAMLformat
    (DATA_DIR "/sample.body-motion.yaml");

  TrajectoryShPtr trajectory;
  {
    ChoreonoidBodyTrajectoryShPtr trajectory_ =
      boost::make_shared<ChoreonoidBodyTrajectory> (bodyMotion, true);
    trajectory = safeGet (trajectory_).trim (0, 10);
  }

  std::string fileMorphing = DATA_DIR;
  fileMorphing += "/human-to-hrp4c.morphing.yaml";
  MorphingData morphing;
  morphing = loadMorphingData (fileMorphing);

  MarkerMappingShPtr mapping =
    buildMarkerMappingFromMorphing (morphing);
  std::cout << safeGet (mapping) << std::endl;

  InteractionMeshShPtr mesh =
    buildInteractionMeshFromMarkerMotion
    (trajectory, mapping);


  boost::shared_ptr<JointToMarkerPositionChoreonoid<
    EigenMatrixDense> >
    jointToMarker =
    boost::make_shared<JointToMarkerPositionChoreonoid<
      EigenMatrixDense> > (robot, morphing);

  Function::vector_t x (trajectory->parameters ().size ());
  x.setZero ();

  boost::shared_ptr<BodyLaplacianDeformationEnergyChoreonoid<
    EigenMatrixDense> >
    cost =
    boost::make_shared<BodyLaplacianDeformationEnergyChoreonoid<
      EigenMatrixDense> >
    (mapping, mesh, trajectory, jointToMarker);

  x.setIdentity ();

  std::cout << "Cost Function Display" << '\n';
  std::cout << (*cost) << '\n';
  for (std::size_t p = 0;
       p < std::min (cost->laplacianCoordinate ().size (), 10UL); ++p)
    {
      StableTimePoint t =
	1. / cost->laplacianCoordinate ().size () * p * tMax;

      Function::vector_t jointPositions = safeGet (trajectory) (t);
      Function::vector_t markerPositions =
	safeGet (jointToMarker) (jointPositions);
      std::cout
	<< "Point " << p << '\n'

	<< "Joint to Marker\n"
	<< markerPositions << '\n'
	<< "Joint to Marker Jacobian\n"
	<< jointToMarker->jacobian (jointPositions) << '\n'

	<< "Laplacian Coordinates\n"
	<< (*cost->laplacianCoordinate ()[p])
	(markerPositions)
	<< '\n'
	<< "Laplacian Coordinates Jacobian\n"
	<< cost->laplacianCoordinate ()[p]->jacobian (markerPositions)
	<< '\n'

	<< "Chain LC\n"
	<< (*cost->chainLc ()[p])
	(jointPositions)
	<< '\n'
	<< "Chain LC Jacobian\n"
	<< cost->chainLc ()[p]->jacobian
	(jointPositions)
	<< '\n'

	<< "LDE\n"
	<< (*cost->lde ()[p])
	((*cost->chainLc ()[p])
	 (jointPositions))
	<< '\n'
        << "LDE Jacobian\n"
	<< (cost->lde ()[p])->jacobian
	((*cost->chainLc ()[p])
	 (jointPositions))
        << '\n'

	<< "Chain\n"
	<< (*cost->chain ()[p])
	(jointPositions)
	<< '\n'
	<< "Chain Jacobian\n"
	<< cost->chain ()[p]->jacobian
	(jointPositions)
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
  // Loading robot.
  cnoid::BodyLoader loader;
  cnoid::BodyPtr robot = loader.load (modelFilePath);
  if (!robot)
    throw std::runtime_error ("failed to load model");

  // Loading the motion.
  cnoid::BodyMotionPtr bodyMotion = boost::make_shared<cnoid::BodyMotion> ();

  bodyMotion->loadStandardYAMLformat
    (DATA_DIR "/sample.body-motion.yaml");

  TrajectoryShPtr trajectory;
  {
    ChoreonoidBodyTrajectoryShPtr trajectory_ =
      boost::make_shared<ChoreonoidBodyTrajectory> (bodyMotion, true);
    trajectory = safeGet (trajectory_).trim (0, 10);
  }

  std::string fileMorphing = DATA_DIR;
  fileMorphing += "/human-to-hrp4c.morphing.yaml";
  MorphingData morphing;
  morphing = loadMorphingData (fileMorphing);

  MarkerMappingShPtr mapping =
    buildMarkerMappingFromMorphing (morphing);

  InteractionMeshShPtr mesh =
    buildInteractionMeshFromMarkerMotion
    (trajectory, mapping);


  std::size_t cfgSize =
    static_cast<std::size_t> (trajectory->outputSize ());
  std::vector<bool> enabledDofs (cfgSize, true);

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


  std::size_t nFrames =
    static_cast<std::size_t>
    (safeGet (trajectory).parameters ().size ()
     / safeGet (trajectory).outputSize ());

  std::size_t nEnabledDofs =
    static_cast<std::size_t>
    (std::count (enabledDofs.begin (), enabledDofs.end (), true));

  // X vector (full form)
  Function::vector_t x (safeGet (trajectory).parameters ().size ());
  x.setZero ();

  // X vector (reduced form)
  Function::vector_t xReduced
    (static_cast<Function::size_type> (nFrames * nEnabledDofs));
  xReduced.setZero ();


  typedef roboptim::Function::value_type value_type;
  std::vector<boost::optional<value_type> > boundDofs
    (cfgSize);
  for (std::size_t jointId = 0; jointId < cfgSize; ++jointId)
    if (!enabledDofs[jointId])
      boundDofs[jointId] = x[static_cast<Function::size_type> (jointId)];

  std::vector<boost::optional<value_type> > boundDofsAllFrames
    (nFrames * cfgSize);
  for (std::size_t frame = 0; frame < nFrames; ++frame)
    for (std::size_t jointId = 0;
	 jointId < cfgSize; ++jointId)
      if (!enabledDofs[jointId])
	boundDofsAllFrames[frame * cfgSize + jointId]
	  = x[static_cast<Function::size_type> (jointId)];

  boost::shared_ptr<JointToMarkerPositionChoreonoid<
    EigenMatrixDense> >
    jointToMarker =
    boost::make_shared<JointToMarkerPositionChoreonoid<
      EigenMatrixDense> > (robot, morphing);

  boost::shared_ptr<BodyLaplacianDeformationEnergyChoreonoid<
    EigenMatrixDense> >
    cost =
    boost::make_shared<BodyLaplacianDeformationEnergyChoreonoid<
      EigenMatrixDense> > (mapping, mesh, trajectory, jointToMarker);

  boost::shared_ptr<DifferentiableFunction>
    costFiltered =
    roboptim::bind<DifferentiableFunction> (cost, boundDofsAllFrames);

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
