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

#include <roboptim/retargeting/function/torque/metapod.hh>

#define BOOST_TEST_MODULE torque_metapod

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

using boost::test_tools::output_test_stream;

using namespace roboptim;

BOOST_AUTO_TEST_CASE (rnd)
{
  typedef metapod::hrp4g2<double> robot_t;

  typedef retargeting::TorqueMetapod<
    EigenMatrixDense, robot_t>::vector_t vector_t;
  retargeting::TorqueMetapod<EigenMatrixDense, robot_t> torque;

  vector_t x (3 * robot_t::NBDOF);
  x.setZero ();

  // check that the center of mass and the Torque are at the same
  // position if velocity and acceleration is null.
  for (int i = 0; i < 100; ++i)
    {
      x.segment (0, robot_t::NBDOF) = vector_t::Random (robot_t::NBDOF);
      vector_t res = torque (x);

      std::cout << "----------" << iendl
		<< "X:" << incindent << iendl
		<< x << decindent << iendl
		<< "Torque(X): " << incindent << iendl
		<< res << decindent << iendl;
    }

    std::cout << "==========" << iendl;

    for (int i = 0; i < 100; ++i)
    {
      x = vector_t::Random (3 * robot_t::NBDOF);
      vector_t res = torque (x);

      std::cout << "----------" << iendl
		<< "X:" << incindent << iendl
		<< x << decindent << iendl
		<< "Torque(X): " << incindent << iendl
		<< res << decindent << iendl;
    }

}
