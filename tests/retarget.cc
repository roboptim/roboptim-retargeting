#include <boost/format.hpp>
#include <log4cxx/logger.h>
#include <roboptim/retargeting/retarget.hh>

#define BOOST_TEST_MODULE retarget

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <cnoid/BodyLoader>

#include "tests-config.h"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (simple)
{
  configureLog4cxx ();

  log4cxx::LoggerPtr logger
    (log4cxx::Logger::getLogger("roboptim.retargeting.tests.retarget"));

  std::string trajectoryFile = TESTS_DATA_DIR;
  trajectoryFile += "/data/dance_one-frame.yaml";

  std::string characterFile = TESTS_DATA_DIR;
  characterFile += "/data/character-cgvu-hrp4c.yaml";

  // Loading character.
  cnoid::CharacterPtr character (new cnoid::Character ());
  character->load (characterFile, std::cout);

  // Loading marker motion.
  cnoid::MarkerMotionPtr markerMotion (new cnoid::MarkerMotion ());
  markerMotion->loadStdYAMLformat (trajectoryFile);

  // Loading robot.
  cnoid::BodyLoader loader;
  cnoid::BodyPtr body = loader.load ("/home/moulard/HRP4C-release/HRP4Cg2.yaml");
  if (!body)
    throw std::runtime_error ("failed to load model");


  roboptim::retargeting::Retarget retarget
    (markerMotion,
     character,
     body,
     false,
     false,
     false,
     false,
     false,
     "ipopt-sparse");
  retarget.animatedMesh ()->writeGraphvizGraphs ("/tmp");

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
