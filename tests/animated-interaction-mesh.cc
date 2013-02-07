#include <string>
#include <fstream>

#include <boost/format.hpp>
#include <log4cxx/logger.h>
#include <roboptim/retargeting/animated-interaction-mesh.hh>

#define BOOST_TEST_MODULE animated_interaction_mesh

#include <boost/test/output_test_stream.hpp>
#include <boost/test/unit_test.hpp>

#include "tests-config.h"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (load_and_recreate)
{
  typedef roboptim::retargeting::AnimatedInteractionMesh
    AnimatedInteractionMesh;
  configureLog4cxx ();

  log4cxx::LoggerPtr logger
    (log4cxx::Logger::getLogger
     ("roboptim.retargeting.tests.animated-interaction-mesh"));

  std::string trajectoryFile = TESTS_DATA_DIR;
  trajectoryFile += "/data/dance_ten-frames.yaml";

  std::string characterFile = TESTS_DATA_DIR;
  characterFile += "/data/character-cgvu-hrp4c.yaml";

  roboptim::retargeting::AnimatedInteractionMeshShPtr_t animatedMesh =
    AnimatedInteractionMesh::loadAnimatedMesh
    (trajectoryFile, characterFile);

  Eigen::VectorXd x = animatedMesh->makeOptimizationVector ();

  roboptim::retargeting::AnimatedInteractionMeshShPtr_t animatedMesh2 =
    AnimatedInteractionMesh::makeFromOptimizationVariables
    (x, animatedMesh);
  Eigen::VectorXd x2 = animatedMesh2->makeOptimizationVector ();

  BOOST_CHECK_EQUAL_COLLECTIONS
    (&x[0], &x[x.size () - 1],
     &x2[0], &x2[x2.size () - 1]);

  BOOST_CHECK_EQUAL
    (animatedMesh->numVertices (), animatedMesh2->numVertices ());
  BOOST_CHECK_EQUAL
    (animatedMesh->numFrames (), animatedMesh2->numFrames ());

  BOOST_CHECK_EQUAL
    (boost::num_vertices (animatedMesh->graph ()),
     boost::num_vertices (animatedMesh2->graph ()));
  BOOST_CHECK_EQUAL
    (boost::num_edges (animatedMesh->graph ()),
     boost::num_edges (animatedMesh2->graph ()));

  BOOST_CHECK_EQUAL
    (&animatedMesh->graph ()[0].positions[0][0],
     &animatedMesh->state ()[0]);

  BOOST_CHECK_EQUAL
    (&animatedMesh->graph ()[0].positions[0][1],
     &animatedMesh->state ()[1]);

  BOOST_CHECK_EQUAL
    (&animatedMesh->graph ()[0].positions[0][2],
     &animatedMesh->state ()[2]);

  for (unsigned i = 0; i < animatedMesh->numFrames (); ++i)
    {
      output_test_stream stream;
      output_test_stream stream2;
      animatedMesh->writeGraphvizGraphs (stream, i);
      animatedMesh2->writeGraphvizGraphs (stream2, i);
      BOOST_CHECK (stream.str () == stream2.str ());

      std::ofstream out ("/tmp/graph1.dot");
      out << stream.str ();
      std::ofstream out2 ("/tmp/graph2.dot");
      out2 << stream2.str ();
    }
}
