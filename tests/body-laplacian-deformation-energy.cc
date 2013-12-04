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

  std::vector<bool> enabledDofs (6 + 44, true);
  std::size_t nEnabledDofs =
    std::count (enabledDofs.begin (), enabledDofs.end (), true);

  Function::vector_t x (bodyMotion->numFrames () * nEnabledDofs);
  x.setZero ();

  boost::shared_ptr<JointToMarkerPositionChoreonoid<
    EigenMatrixDense> >
    jointToMarker =
    boost::make_shared<JointToMarkerPositionChoreonoid<
      EigenMatrixDense> > (mesh, 0);

  boost::shared_ptr<BodyLaplacianDeformationEnergyChoreonoid<
    EigenMatrixDense> >
    cost =
    boost::make_shared<BodyLaplacianDeformationEnergyChoreonoid<
      EigenMatrixDense> >
    (mesh, nEnabledDofs, bodyMotion->getNumFrames (), x,
     jointToMarker, jointToMarker);

  std::cout << (*cost) (x) << std::endl;
  std::cout << (*cost) (x) << std::endl;
  std::cout << (*cost) (x) << std::endl;
  std::cout << (*cost) (x) << std::endl;
  std::cout << (*cost) (x) << std::endl;
  std::cout << (*cost) (x) << std::endl;
  std::cout << (*cost) << std::endl;
}
