#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <roboptim/retargeting/retarget.hh>

#include <cnoid/ItemTreeView>
#include <cnoid/LazyCaller>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/Plugin>
#include <cnoid/RootItem>

#include <cnoid/src/MocapPlugin/MarkerMotionItem.h>

#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>

#include "directories.hh"

class RoboptimRetargetingPlugin : public cnoid::Plugin
{
public:
  explicit RoboptimRetargetingPlugin ()
    : Plugin("RoboptimRetargeting"),
      thread_ ()
  {

  }

  virtual bool initialize ()
  {
    // Initialize logging system.
    std::string log4cxxConfigurationFile = PKG_SHARE_DIR;
    log4cxxConfigurationFile += "/log4cxx.xml";
    log4cxx::xml::DOMConfigurator::configure (log4cxxConfigurationFile);

    cnoid::Action* menuItem = menuManager ().
      setPath ("/Filters").addItem ("Roboptim Retargeting");
    menuItem->sigTriggered ().connect
      (boost::bind (&RoboptimRetargetingPlugin::menuCb, this));
    return true;
  }

private:
  void menuCb ()
  {
    if (thread_)
      {
	cnoid::MessageView::mainInstance ()->putln
	  ("Roboptim Retargeting - thread already launched, ignoring command");
	return;
      }

    cnoid::MessageView::mainInstance ()->putln ("Roboptim Retargeting - launching thread");
    thread_ = boost::make_shared<boost::thread>
      (boost::bind (&RoboptimRetargetingPlugin::buildProblemAndOptimize, this));
  }

  void buildProblemAndOptimize ()
  {
    cnoid::MessageView::mainInstance ()->putln ("Roboptim Retargeting - start");


    log4cxx::LoggerPtr logger
      (log4cxx::Logger::getLogger
       ("roboptim.retargeting.choreonoid"));

    std::string trajectoryFilePath ("/home/moulard/29_07-markers-hrp4c.yaml");
    std::string characterFilePath
      ("/home/moulard/profiles/default-x86_64-linux-ubuntu-12.04.1/src/unstable/"
       "aist/choreonoid/ext/MocapPlugin/TestData/character-cmu-hrp4c.yaml");
    std::string modelFilePath ("/home/moulard/HRP4C-release/HRP4Cg2.yaml");
    bool enableBoneLength = true;
    bool enablePosition = false;
    bool enableCollision = false;
    bool enableTorque = false;
    bool enableZmp = false;
    std::string solverName = "ipopt-sparse";

    // Retarget motion.
    roboptim::retargeting::Retarget retarget
      (trajectoryFilePath,
       characterFilePath,
       modelFilePath,
       enableBoneLength,
       enablePosition,
       enableCollision,
       enableTorque,
       enableZmp,
       solverName);

    retarget.solve ();

    cnoid::MarkerMotionItemPtr resultItem = new cnoid::MarkerMotionItem ();
    resultItem->setName("roboptim-retargeting-result");

    // ask main thread to add the item.
    cnoid::callLater
      (boost::bind (&cnoid::RootItem::addChildItem,
		    cnoid::ItemTreeView::instance()->rootItem (),
		    resultItem,
		    false));

    // Get the result.
    Eigen::VectorXd x;

    const std::string filename ("/tmp/result.yaml");

    // Populate result.
    // Check if the minimization has succeed.
    if (retarget.result ().which () ==
	roboptim::retargeting::Retarget::solver_t::SOLVER_ERROR)
      {
	const roboptim::SolverError& result =
	  boost::get<roboptim::SolverError> (retarget.result ());

	if (result.lastState ())
	  {
	    retarget.animatedMesh ()->state () = result.lastState ()->x;
	    retarget.animatedMesh ()->computeVertexWeights ();
	    retarget.animatedMesh ()->writeTrajectory (filename);
	    LOG4CXX_INFO (logger, "last state: " << result);
	    LOG4CXX_INFO (logger, "trajectory written to: " << filename);
	  }

	std::cout << "No solution has been found. Failing..."
		  << std::endl
		  << boost::get<roboptim::SolverError>
	  (retarget.result ()).what ()
		  << std::endl;
	return;
      }

    LOG4CXX_INFO (logger, "a solution has been found!");

    if (retarget.result ().which () ==
	roboptim::retargeting::Retarget::solver_t::SOLVER_VALUE_WARNINGS)
      {
	const roboptim::Result& result =
	  boost::get<roboptim::ResultWithWarnings> (retarget.result ());
	x = result.x;
	LOG4CXX_WARN (logger, "solver warnings: " << result);
      }
    else
      {
	const roboptim::Result& result =
	  boost::get<roboptim::Result> (retarget.result ());
	x = result.x;
	LOG4CXX_WARN (logger, "result: " << result);
      }


    retarget.animatedMesh ()->state () = x;
    retarget.animatedMesh ()->computeVertexWeights ();
    retarget.animatedMesh ()->writeTrajectory (filename);
    LOG4CXX_INFO (logger, "trajectory written to: " << filename);

    resultItem->seq ()->setFrameRate (retarget.animatedMesh ()->framerate ());
    resultItem->seq ()->setNumParts (retarget.animatedMesh ()->numVertices ());
    resultItem->seq ()->setNumFrames (retarget.animatedMesh ()->numFrames ());

    for (int frameId = 0; frameId < retarget.animatedMesh ()->numFrames (); ++frameId)
      for (std::size_t markerId = 0;
	   markerId < retarget.animatedMesh ()->numVertices (); ++markerId)
      resultItem->seq ()->frame (frameId)[markerId] =
	x.segment (frameId * retarget.animatedMesh ()->optimizationVectorSizeOneFrame () + markerId, 3);

    cnoid::MessageView::mainInstance ()->putln ("Roboptim Retargeting - end");
  }

  boost::shared_ptr<boost::thread> thread_;
};

CNOID_IMPLEMENT_PLUGIN_ENTRY (RoboptimRetargetingPlugin)
