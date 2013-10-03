#include <roboptim/retargeting/function/zmp/choreonoid.hh>

#include <cnoid/BodyLoader>

#define BOOST_TEST_MODULE zmp_chorenoid

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include "tests-config.h"

using boost::test_tools::output_test_stream;

using namespace roboptim;

BOOST_AUTO_TEST_CASE (rnd)
{
  configureLog4cxx ();

  //FIXME: we should embed the copy.
  std::string modelFilePath
    ("/home/moulard/HRP4C-release/HRP4Cg2.yaml");

  // Loading robot.
  cnoid::BodyLoader loader;
  cnoid::BodyPtr robot = loader.load (modelFilePath);
  if (!robot)
    throw std::runtime_error ("failed to load model");

  typedef retargeting::ZMPChoreonoid<
    EigenMatrixDense>::vector_t vector_t;
  retargeting::ZMPChoreonoid<EigenMatrixDense> zmp (robot);

  vector_t x (3 * robot->numJoints ());
  x.setZero ();

  // check that the center of mass and the ZMP are at the same
  // position if velocity and acceleration is null.
  for (int i = 0; i < 100; ++i)
    {
      x.segment (0, robot->numJoints ()) = vector_t::Random (robot->numJoints ());
      vector_t res = zmp (x);

      std::cout << "----------" << iendl
		<< "X:" << incindent << iendl
		<< x << decindent << iendl
		<< "ZMP(X): " << incindent << iendl
		<< res << decindent << iendl;

      robot->calcCenterOfMass ();
      BOOST_CHECK_EQUAL (res[0], robot->centerOfMass ()[0]);
      BOOST_CHECK_EQUAL (res[1], robot->centerOfMass ()[1]);
    }
}
