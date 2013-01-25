#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/program_options.hpp>

#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>

#include <roboptim/retargeting/retarget.hh>

#include <roboptim/retargeting/config.hh>
#include "directories.hh"

void help (const boost::program_options::options_description& desc)
{
  std::cout
    << "usage: "
    << "roboptim-retarget-motion -t TRAJECTORY_FILE -r ROBOT_FILE\n\n"
    << desc << std::endl;
}

void version ()
{
  std::cout << "roboptim-retarget-motion v "
	    << ROBOPTIM_RETARGETING_VERSION << std::endl;
}

int main (int argc, char** argv)
{
  namespace fs = boost::filesystem;
  namespace po = boost::program_options;

  // Initialize logging system.
  std::string log4cxxConfigurationFile = PKG_SHARE_DIR;
  log4cxxConfigurationFile += "/log4cxx.xml";
  log4cxx::xml::DOMConfigurator::configure (log4cxxConfigurationFile);

  log4cxx::LoggerPtr logger
    (log4cxx::Logger::getLogger
     ("roboptim.retargeting.roboptim-retarget-motion"));

  // Parse options.
  std::string trajectoryFile;
  std::string characterFile;

  po::options_description desc ("Allowed options");
  desc.add_options ()
    ("help,h", "produce help message")
    ("version,v", "print version string")
    ("trajectory,t", po::value<std::string> (&trajectoryFile),
     "markers trajectory (YAML)")
    ("robot,r", po::value<std::string> (&characterFile),
     "robot description (YAML)")
    ;

  po::variables_map vm;
  try
    {
      po::store(po::parse_command_line (argc, argv, desc), vm);
      po::notify (vm);
    }
  catch (po::error& e)
    {
      std::cout << e.what () << std::endl;
      help (desc);
      return 5;
    }

  if (vm.count ("help"))
    {
      help (desc);
      return 0;
    }

  if (vm.count ("version"))
    {
      version ();
      return 0;
    }

  // Check options.
  if (trajectoryFile.empty () && characterFile.empty ())
    {
      std::cout << "trajectory and robot files are missing" << std::endl;
      help (desc);
      return 1;
    }
  if (trajectoryFile.empty () && !characterFile.empty ())
    {
      std::cout << "trajectory file is missing" << std::endl;
      help (desc);
      return 1;
    }
  if (!trajectoryFile.empty () && characterFile.empty ())
    {
      std::cout << "robot file is missing" << std::endl;
      help (desc);
      return 1;
    }

  // Resolve files.
  fs::path dataDir (PKG_SHARE_DIR);
  dataDir /= "data";

  fs::path trajectoryFilePath (trajectoryFile);
  fs::path characterFilePath (characterFile);

  if (!fs::exists (trajectoryFilePath))
    {
      trajectoryFilePath = dataDir / trajectoryFilePath;
      if (!fs::exists (trajectoryFilePath))
	{
	  std::cerr << "trajectory file does not exist" << std::endl;
	  return 2;
	}
    }

  if (!fs::exists (characterFilePath))
    {
      characterFilePath = dataDir / characterFilePath;
      if (!fs::exists (characterFilePath))
	{
	  std::cerr << "robot file does not exist" << std::endl;
	  return 2;
	}
    }

  LOG4CXX_INFO (logger, "loading optimization problem...");

  // Retarget motion.
  roboptim::retargeting::Retarget retarget
    (trajectoryFilePath.string (),
     characterFilePath.string ());

  LOG4CXX_DEBUG (logger,
		"Problem:\n" << *retarget.problem ());

  LOG4CXX_INFO (logger, "solving optimization problem...");
  retarget.solve ();
  LOG4CXX_INFO (logger, "done");

  // Check if the minimization has succeed.
  if (retarget.result ().which () !=
      roboptim::retargeting::Retarget::solver_t::SOLVER_VALUE)
    {
      std::cout << "A solution should have been found. Failing..."
                << std::endl
                << boost::get<roboptim::SolverError>
	(retarget.result ()).what ()
                << std::endl;
      return 10;
    }

  // Get the result.
  const roboptim::Result& result =
    boost::get<roboptim::Result> (retarget.result ());

  // Display the result.
  std::cout << "A solution has been found: " << std::endl;
  std::cout << result << std::endl;
}
