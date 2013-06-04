#include <fstream>

#include <roboptim/core/finite-difference-gradient.hh>
#include <roboptim/retargeting/inverse-kinematics.hh>
#include <roboptim/core/visualization/gnuplot.hh>
#include <roboptim/core/visualization/gnuplot-commands.hh>

#define BOOST_TEST_MODULE inverse_kinematics

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include "tests-config.h"

using boost::test_tools::output_test_stream;

using namespace roboptim;
using namespace roboptim::retargeting;

static double deg2rad (double x)
{
  return x * M_PI / 360.;
}

BOOST_AUTO_TEST_CASE (simple)
{
  std::string hrpModel
    ("/home/moulard/HRP4C-release/HRP4Cg2main.wrl");

  configureLog4cxx ();

  log4cxx::LoggerPtr logger
    (log4cxx::Logger::getLogger("roboptim.retargeting.tests.ik"));

  boost::shared_ptr<InverseKinematics> ik
    = InverseKinematics::create (hrpModel);

  // Reference configuration.
  InverseKinematics::argument_t q (ik->model ()->numJoints ());
  q.setZero ();

  q <<
    0., 0., deg2rad (-25.), deg2rad (50.), deg2rad (-25.), 0., 0.,
    0., 0., deg2rad (-25.), deg2rad (50.), deg2rad (-25.), 0., 0.,
    0., 0., 0.,
    0., 0., 0.,
    deg2rad (-1.), deg2rad (1.), 0., 0., 0., deg2rad (-1.), deg2rad (1.), deg2rad (-1.),
    deg2rad (5.), deg2rad (-10.), 0., deg2rad (-15.), 0.,  deg2rad (10.),  deg2rad (1.), 0.,
    deg2rad (5.), deg2rad (10.), 0., deg2rad (-15.), 0., deg2rad (-10.), deg2rad (-1.), 0.;

  std::cout << "q: " << q << std::endl;

  // Set the configuration.
  for (std::size_t jointId = 0; jointId < ik->model ()->numJoints (); ++jointId)
    {
      hrp::Link *link = ik->model ()->joint (jointId);
      if (link)
	link->q = q[jointId];
    }
  ik->model ()->calcForwardKinematics();


  // Compute associated body position.
  InverseKinematics::argument_t bodyPositions (ik->inputSize ());
  bodyPositions.setZero ();

  for (std::size_t linkId = 0; linkId < ik->model ()->numLinks (); ++linkId)
    {
      hrp::Link *link = ik->model ()->link (linkId);
      assert (link);
      bodyPositions.segment (linkId * 3, 3) = link->p;
    }

  Eigen::Vector3d zero = Eigen::Vector3d::Zero ();

  // Solve IK
  InverseKinematics::result_t qIk = (*ik) (bodyPositions);

  roboptim::visualization::Gnuplot gp =
    roboptim::visualization::Gnuplot::make_interactive_gnuplot ();

  std::ofstream f ("/tmp/body-positions.gp");
  f << gp << std::endl
	    << "splot '-' w labels ls 9\n";

  typedef std::vector<hrp::Link*>::const_iterator it_t;
  it_t it = ik->model ()->links ().begin ();
  for (std::size_t bodyId = 0; bodyId < ik->model ()->numLinks (); ++bodyId, ++it)
    {
      f << bodyPositions[bodyId * 3 + 0] << " "
	<< bodyPositions[bodyId * 3 + 1] << " "
	<< bodyPositions[bodyId * 3 + 2] << " "
	<< "\"" << (*it)->name << "\"\n";
    }
  f << "e" << std::endl;

  std::cout << "q (after ik); " << qIk << std::endl;

  for (std::size_t jointId = 0; jointId < qIk.size (); ++jointId)
    {
      BOOST_CHECK_CLOSE (1. + q[jointId], 1. + qIk[jointId], 5.);
    }
}
