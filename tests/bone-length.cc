#include <roboptim/core/finite-difference-gradient.hh>
#include <roboptim/retargeting/bone-length.hh>
#include <roboptim/retargeting/animated-interaction-mesh.hh>

#define BOOST_TEST_MODULE bone_length

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include "tests-config.h"

using boost::test_tools::output_test_stream;

using namespace roboptim;
using namespace roboptim::retargeting;

BOOST_AUTO_TEST_CASE (simple)
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

  AnimatedInteractionMeshShPtr_t animatedMeshLocal =
    AnimatedInteractionMesh::makeFromOptimizationVariables
    (animatedMesh->state (), animatedMesh);

  AnimatedInteractionMesh::edge_iterator_t edgeIt;
  AnimatedInteractionMesh::edge_iterator_t edgeEnd;
  boost::tie(edgeIt, edgeEnd) =
    boost::edges (animatedMesh->graph ());

  AnimatedInteractionMesh::edge_descriptor_t edgeId = *edgeIt;

  BoneLength bl
    (animatedMesh,
     animatedMeshLocal,
     edgeId);

  Function::vector_t x (bl.inputSize ());

  x.setZero ();
  std::cout << "zero: " << std::endl << bl (x) << std::endl;
  x.setConstant (1.);
  std::cout << "one: " << std::endl << bl (x) << std::endl;

  for (unsigned i = 0; i < 1; ++i)
    {
      BOOST_TEST_MESSAGE ("Check gradient, i-th times, i = " << i);
      x.setRandom ();
      BOOST_CHECK_NO_THROW
	(
	 try
	   {
	     checkGradientAndThrow (bl, 0, x);
	   }
	 catch (BadGradient& bg)
	   {
	     BOOST_TEST_MESSAGE (bg);
	     throw;
	   });
    }
}
