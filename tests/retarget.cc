#include <boost/format.hpp>
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
  trajectoryFile += "/data/dance_one-frame.yaml";

  std::string characterFile = TESTS_DATA_DIR;
  characterFile += "/data/character-cgvu-hrp4c.yaml";

  roboptim::retargeting::Retarget retarget
    (trajectoryFile,
     characterFile);
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
