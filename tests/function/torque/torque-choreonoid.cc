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

#include <boost/make_shared.hpp>

#include <roboptim/retargeting/function/minimum-jerk-trajectory.hh>
#include <roboptim/retargeting/function/torque/choreonoid.hh>
#include <roboptim/trajectory/vector-interpolation.hh>

#include <cnoid/BodyLoader>

#define BOOST_TEST_MODULE torque_chorenoid

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

using boost::test_tools::output_test_stream;

using namespace roboptim;
using namespace roboptim::retargeting;

std::string modelFilePath (HRP4C_YAML_FILE);

BOOST_AUTO_TEST_CASE (rnd)
{
  // Loading robot.
  cnoid::BodyLoader loader;
  cnoid::BodyPtr robot = loader.load (modelFilePath);
  if (!robot)
    throw std::runtime_error ("failed to load model");

  typedef TorqueChoreonoid<EigenMatrixDense>::vector_t vector_t;
  TorqueChoreonoid<EigenMatrixDense> torque (robot);

  vector_t x ((6 + robot->numJoints ()) * 3);
  x.setZero ();

  vector_t res;

  for (int i = 0; i < 100; ++i)
    {
      x.setZero ();
      torque.q (x).setRandom ();
      res = torque (x);

      std::cout << "----------" << iendl
		<< "X:" << incindent << iendl
		<< x << decindent << iendl
		<< "Torque(X): " << incindent << iendl
		<< res << decindent << iendl;
      std::cerr << iendl;
    }
}
