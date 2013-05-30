#include <fstream>

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_RUNTIME_NO_MALLOC
#include <rbdl/rbdl.h>
#include <rbdl/Kinematics.h>
#include <rbdl/addons/urdfreader/rbdl_urdfreader.h>

#include <urdf_parser/urdf_parser.h>

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

typedef std::map<std::string, boost::shared_ptr<urdf::Link> >::const_iterator
iter_t;
typedef std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator
joint_iter_t;


static double deg2rad (double x)
{
  return x * M_PI / 360.;
}

BOOST_AUTO_TEST_CASE (simple)
{
  std::string urdfPath
    ("/home/moulard/profiles/default-x86_64-linux-ubuntu-12.04.1/"
     "src/unstable/ros/stacks/hrp2/hrp4_description/urdf/hrp4.urdf");

  std::string xmlString;
  std::fstream xmlFile (urdfPath.c_str (), std::fstream::in);
  while (xmlFile.good ())
    {
      std::string line;
      std::getline (xmlFile, line);
      xmlString += (line + "\n");
    }

  configureLog4cxx ();

  log4cxx::LoggerPtr logger
    (log4cxx::Logger::getLogger("roboptim.retargeting.tests.ik"));

  boost::shared_ptr<urdf::ModelInterface> model =
    urdf::parseURDF (xmlString);
  InverseKinematics ik (urdfPath, model);

  // Reference configuration.
  InverseKinematics::argument_t q (model->joints_.size ());
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

  // Compute associated body position.
  InverseKinematics::argument_t bodyPositions (ik.inputSize ());
  bodyPositions.setZero ();

  RigidBodyDynamics::Math::Vector3d zero =
    RigidBodyDynamics::Math::Vector3d::Zero ();

  for (std::size_t bodyId = 0; bodyId < model->links_.size (); ++bodyId)
    {
      bodyPositions.segment (3 * bodyId, 3) =
	RigidBodyDynamics::CalcBaseToBodyCoordinates
	(ik.rbdlModel (), q, bodyId, zero);
    }

  std::cout << "body positions: " << bodyPositions << std::endl;

  // Solver IK.
  InverseKinematics::result_t qIk = ik (bodyPositions);

  // checking models consistency
  iter_t it = model->links_.begin ();
  for (std::size_t bodyId = 0; bodyId < model->links_.size (); ++bodyId, ++it)
    {
      BOOST_CHECK_EQUAL (it->second->name,
			 ik.rbdlModel ().GetBodyName (bodyId));
    }


  std::cout << "configuration (from ik): " << qIk << std::endl;

  joint_iter_t itJoint = model->joints_.begin ();
  for (std::size_t i = 0; i < q.size (); ++i, ++itJoint)
    {
      std::cout << "jointId: " << i
		<< " " << itJoint->second->name << std::endl;
      BOOST_CHECK_CLOSE (1. + q[i], 1. + qIk[i], 1e1);
    }

  roboptim::visualization::Gnuplot gp =
    roboptim::visualization::Gnuplot::make_interactive_gnuplot ();

  std::ofstream f ("/tmp/body-positions.gp");
  f << gp << std::endl
	    << "splot '-' w labels p ls 9\n";

  it = model->links_.begin ();
  for (std::size_t bodyId = 0; bodyId < model->links_.size (); ++bodyId, ++it)
    {
      f << bodyPositions[bodyId * 3 + 0] << " "
	<< bodyPositions[bodyId * 3 + 1] << " "
	<< bodyPositions[bodyId * 3 + 2] << " "
	<< "\"" << it->second->name << "\"\n";
    }
  f << "e" << std::endl;
}
