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
  configureLog4cxx ();

  std::string trajectoryFile = TESTS_DATA_DIR;
  trajectoryFile += "/dance_longer-markers.yaml";

  std::string characterFile = TESTS_DATA_DIR;
  characterFile += "/character-cgvu-hrp4c.yaml";

  roboptim::retargeting::Retarget retarget
    (trajectoryFile,
     characterFile);

  Eigen::Matrix<double, Eigen::Dynamic, 1> x
    (retarget.animatedMesh ()->optimizationVectorSize ());
  x.setZero ();

  std::cout << "Cost (laplacian): "
	    << (*retarget.costLaplacian ())(x) << std::endl;
  std::cout << "Cost (acceleration): "
	    << (*retarget.costAcceleration ())(x) << std::endl;
  std::cout << "Cost: " << (*retarget.cost ())(x) << std::endl;

  std::cout << "Problem:\n" << *retarget.problem () << std::endl;

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
