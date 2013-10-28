#include <boost/make_shared.hpp>

#include <roboptim/retargeting/function/minimum-jerk-trajectory.hh>
#include <roboptim/retargeting/function/torque/choreonoid.hh>
#include <roboptim/trajectory/vector-interpolation.hh>

#include <cnoid/BodyLoader>

#define BOOST_TEST_MODULE torque_chorenoid

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include "tests-config.h"

using boost::test_tools::output_test_stream;

using namespace roboptim;
using namespace roboptim::retargeting;

//FIXME: we should embed the copy.
std::string modelFilePath
("/home/moulard/HRP4C-release/HRP4Cg2.yaml");


BOOST_AUTO_TEST_CASE (rnd)
{
  configureLog4cxx ();

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
