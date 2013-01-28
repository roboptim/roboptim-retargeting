#include <string>

#include <boost/format.hpp>
#include <log4cxx/logger.h>
#include <roboptim/retargeting/interaction-mesh.hh>

#define BOOST_TEST_MODULE interaction_mesh

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include "tests-config.h"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (load_and_recreate)
{
  configureLog4cxx ();

  log4cxx::LoggerPtr logger
    (log4cxx::Logger::getLogger
     ("roboptim.retargeting.tests.interaction-mesh"));

  std::string trajectoryFile = TESTS_DATA_DIR;
  trajectoryFile += "/data/dance_one-frame.yaml";

  roboptim::retargeting::InteractionMeshShPtr_t mesh =
    roboptim::retargeting::InteractionMesh::loadMesh (trajectoryFile, 0);

  Eigen::Matrix<double, Eigen::Dynamic, 1> x =
    mesh->optimizationVector ();

  roboptim::retargeting::InteractionMeshShPtr_t mesh2 =
    roboptim::retargeting::InteractionMesh::makeFromOptimizationVariables
    (x);
 
  BOOST_CHECK_EQUAL (boost::num_vertices (mesh->graph ()),
		     boost::num_vertices (mesh2->graph ())); 
  BOOST_CHECK_EQUAL (boost::num_edges (mesh->graph ()),
		     boost::num_edges (mesh2->graph ())); 
}
