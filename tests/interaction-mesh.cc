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

#define BOOST_TEST_MODULE morphing

#include <boost/test/unit_test.hpp>

#include <libmocap/marker-trajectory-factory.hh>
#include <libmocap/marker-trajectory.hh>

#include <roboptim/retargeting/interaction-mesh.hh>

#include <roboptim/retargeting/function/libmocap-marker-trajectory.hh>


using namespace roboptim;
using namespace roboptim::retargeting;

BOOST_AUTO_TEST_CASE (interaction_mesh)
{
  std::string file = DATA_DIR;
  file += "/human.trc";

  libmocap::MarkerTrajectoryFactory factory;
  libmocap::MarkerTrajectory markers =
    factory.load (file);

  MarkerMappingShPtr mapping =
    buildMarkerMappingFromMotion (markers);

  LibmocapMarkerTrajectoryShPtr trajectory =
    boost::make_shared<LibmocapMarkerTrajectory> (markers);

  InteractionMeshShPtr interactionMesh =
    buildInteractionMeshFromMarkerMotion
    (trajectory, mapping);


  std::cout << *interactionMesh << std::endl;
}
