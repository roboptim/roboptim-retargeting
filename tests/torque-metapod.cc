#include <roboptim/retargeting/function/torque/metapod.hh>

#include "model/hrp4g2.hh"

#define BOOST_TEST_MODULE torque_chorenoid

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include "tests-config.h"

using boost::test_tools::output_test_stream;

using namespace roboptim;

BOOST_AUTO_TEST_CASE (rnd)
{
  configureLog4cxx ();

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
