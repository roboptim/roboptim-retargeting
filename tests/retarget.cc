#include <boost/format.hpp>
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

  for (unsigned i = 0;
       i < retarget.animatedMesh ()->meshes ().size ();
       ++i)
    {
      std::cout << " --- frame " << i << " ---" << std::endl
		<< "Number of vertices: "
		<< boost::num_vertices
	(retarget.animatedMesh ()->meshes ()[i]->graph ()) << std::endl;
      
      std::ofstream graphvizFile
	((boost::format ("/tmp/graph_%1%.dot") % i).str().c_str ());
      boost::write_graphviz
	(graphvizFile, retarget.animatedMesh ()->meshes ()[i]->graph ());
    }

  retarget.solve ();
}
