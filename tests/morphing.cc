#define BOOST_TEST_MODULE morphing

#include <boost/test/unit_test.hpp>

#include <roboptim/retargeting/morphing.hh>

using namespace roboptim;
using namespace roboptim::retargeting;

BOOST_AUTO_TEST_CASE (morphing)
{
  std::string file = DATA_DIR;
  file += "/human-to-hrp4c.morphing.yaml";

  MorphingData morphing;
  morphing = loadMorphingData (file);

}
