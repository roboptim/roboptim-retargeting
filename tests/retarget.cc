#include <boost/format.hpp>
#include <boost/graph/graphviz.hpp>
#include <log4cxx/logger.h>
#include <roboptim/retargeting/retarget.hh>

#define BOOST_TEST_MODULE retarget

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include "tests-config.h"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (simple)
{
  configureLog4cxx ();

  log4cxx::LoggerPtr logger
    (log4cxx::Logger::getLogger("roboptim.retargeting.tests.retarget"));

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

  LOG4CXX_INFO (logger,
		"Cost (laplacian): "
		<< (*retarget.costLaplacian ())(x));
  LOG4CXX_INFO (logger,
		"Cost (acceleration): "
		<< (*retarget.costAcceleration ())(x));
  LOG4CXX_INFO (logger,
		"Cost: " << (*retarget.cost ())(x));

  for (unsigned i = 0;
       i < retarget.animatedMesh ()->meshes ().size ();
       ++i)
    {
      LOG4CXX_DEBUG
	(logger,
	 " --- frame " << i << " ---"
	 << "Number of vertices: "
	 << boost::num_vertices
	 (retarget.animatedMesh ()->meshes ()[i]->graph ()));

      std::ofstream graphvizFile
	((boost::format ("/tmp/graph_%1%.dot") % i).str().c_str ());
      boost::write_graphviz
	(graphvizFile, retarget.animatedMesh ()->meshes ()[i]->graph (),
	 roboptim::retargeting::InteractionMeshGraphVertexWriter<
	   roboptim::retargeting::InteractionMesh::graph_t>
	 (retarget.animatedMesh ()->meshes ()[i]->graph (),
	  retarget.animatedMesh ()->vertexLabels ()),
	 roboptim::retargeting::InteractionMeshGraphEdgeWriter<
	   roboptim::retargeting::InteractionMesh::graph_t>
	 (retarget.animatedMesh ()->meshes ()[i]->graph ()));
    }

  LOG4CXX_INFO (logger,
		"Problem:\n" << *retarget.problem ());

  retarget.solve ();

  // Check if the minimization has succeed.
  if (retarget.result ().which () !=
      roboptim::retargeting::Retarget::solver_t::SOLVER_VALUE)
    {
      std::cout << "A solution should have been found. Failing..."
                << std::endl
                << boost::get<roboptim::SolverError>
	(retarget.result ()).what ()
                << std::endl;
      return;
    }

  // Get the result.
  const roboptim::Result& result =
    boost::get<roboptim::Result> (retarget.result ());

  // Display the result.
  std::cout << "A solution has been found: " << std::endl;
  std::cout << result << std::endl;
}
