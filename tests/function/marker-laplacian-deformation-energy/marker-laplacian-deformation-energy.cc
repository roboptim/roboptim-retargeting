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


#define BOOST_TEST_MODULE choreonoid_marker_trajectory

#include <fstream>

#include <boost/array.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <libmocap/marker-trajectory-factory.hh>
#include <libmocap/marker-trajectory.hh>

#include <roboptim/core/visualization/gnuplot.hh>
#include <roboptim/core/visualization/gnuplot-commands.hh>
#include <roboptim/core/visualization/gnuplot-function.hh>

#include <roboptim/retargeting/interaction-mesh.hh>

#include <roboptim/retargeting/function/libmocap-marker-trajectory.hh>

#include <roboptim/retargeting/function/marker-laplacian-deformation-energy/choreonoid.hh>

using namespace roboptim;
using namespace roboptim::visualization;
using namespace roboptim::retargeting;

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (simple)
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

  InteractionMeshShPtr mesh =
    buildInteractionMeshFromMarkerMotion
    (trajectory, mapping);

  MarkerLaplacianDeformationEnergyChoreonoidDenseShPtr
    cost =
    boost::make_shared<MarkerLaplacianDeformationEnergyChoreonoidDense>
    (mapping, mesh, trajectory);

  Function::vector_t::Index frameSize = safeGet(trajectory).outputSize ();

  Function::vector_t
    x (cost->inputSize ());
  x = safeGet (trajectory).parameters ();

  BOOST_CHECK_EQUAL ((*cost) (x)[0], 0.);

  std::cout << "Cost Function Display" << '\n';

  std::cout << safeGet (cost) << '\n';

  for (std::size_t p = 0;
       p < std::min (cost->laplacianCoordinate ().size (), 10UL); ++p)
    {
      Eigen::VectorBlock<Function::vector_t> frameX =
	x.segment (static_cast<Function::vector_t::Index> (p) * frameSize, frameSize);

      std::cout
	<< "Frame " << p << '\n'

	<< "Laplacian Coordinates\n"
	<< (*cost->laplacianCoordinate ()[p]) (frameX)
	<< '\n'
	<< "Laplacian Coordinates Jacobian\n"
	<< cost->laplacianCoordinate ()[p]->jacobian (frameX)
	<< '\n'
	<< "\n\n\n";
    }

  std::cout << "Marker Laplacian Deformation Energy" << '\n';
  std::cout << (*cost) (x) << '\n';


  std::cout << "Marker Laplacian Deformation Energy Jacobian" << '\n';
  std::cout << cost->jacobian (x) << '\n';

  std::ofstream log ("/tmp/marker-laplacian-deformation-energy-jac-nodisableddofs.txt");
  log << cost->jacobian (x);
}
