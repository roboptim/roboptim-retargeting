#include <boost/graph/graphviz.hpp>
#include <roboptim/retargeting/retarget.hh>

#define BOOST_TEST_MODULE retarget

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include "tests-config.h"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (simple)
{
  std::string trajectoryFile = TESTS_DATA_DIR;
  trajectoryFile += "/dance_longer-markers.yaml";

  roboptim::retargeting::Retarget retarget
    (trajectoryFile);

  std::cout << "Number of vertices: "
	    << boost::num_vertices (retarget.mesh ()->graph ()) << std::endl;

  std::ofstream graphvizFile ("/tmp/graph.dot");
  boost::write_graphviz (graphvizFile, retarget.mesh ()->graph ());

  retarget.solve ();
}
