#include <roboptim/retargeting/acceleration-energy.hh>
#include <roboptim/retargeting/interaction-mesh.hh>

#define BOOST_TEST_MODULE acceleration_energy

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include "tests-config.h"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (simple)
{
  configureLog4cxx ();
}
