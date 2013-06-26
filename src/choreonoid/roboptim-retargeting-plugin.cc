#include <sstream>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <roboptim/retargeting/retarget.hh>

#include <cnoid/BodyItem>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>
#include <cnoid/LazyCaller>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/Plugin>
#include <cnoid/RootItem>

#include <cnoid/src/MocapPlugin/CharacterItem.h>

#define private public
#include <cnoid/src/MocapPlugin/MarkerMotionItem.h>
#undef private

#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>

#include "directories.hh"

class RoboptimRetargetingPlugin : public cnoid::Plugin
{
public:
  explicit RoboptimRetargetingPlugin ()
    : Plugin("RoboptimRetargeting"),
      menuItem_ (),
      thread_ ()
  {

  }

  virtual bool initialize ()
  {
    // Initialize logging system.
    std::string log4cxxConfigurationFile = PKG_SHARE_DIR;
    log4cxxConfigurationFile += "/log4cxx.xml";
    log4cxx::xml::DOMConfigurator::configure (log4cxxConfigurationFile);

    menuItem_ =
      menuManager ().setPath ("/Filters").addItem ("Roboptim Retargeting");
    menuItem_->sigTriggered ().connect
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

    cnoid::BodyPtr body;
    cnoid::CharacterPtr character;
    cnoid::MarkerMotionItemPtr markerMotionItem;

    cnoid::ItemList<cnoid::Item> selected =
      cnoid::ItemTreeView::instance()->selectedItems<cnoid::Item>();
    cnoid::ItemList<cnoid::BodyItem> bodyItems = selected;

    for (int i = 0; i < bodyItems.size (); ++i)
      {
	cnoid::BodyItem* bodyItem = bodyItems.get (i);
	if (bodyItem->body ())
	  body = bodyItem->body ();

	for (int j = 0; j < selected.size (); ++j)
	  {
	    cnoid::MarkerMotionItem* markerMotionItemTmp =
	      dynamic_cast<cnoid::MarkerMotionItem*> (selected.get (j));
	    if (markerMotionItemTmp)
	      {
		markerMotionItem = markerMotionItemTmp;
		character = markerMotionItem->character ();
	      }
	  }
      }

    if (!body)
      {
	cnoid::MessageView::mainInstance ()->putln
	  ("no robot model item is selected");
	return;
      }

    if (!markerMotionItem)
      {
	cnoid::MessageView::mainInstance ()->putln
	  ("no marker motion is selected");
	return;
      }

    if (!character)
      {
	cnoid::MessageView::mainInstance ()->putln
	  ("no character attached to marker motion");
	return;
      }

    cnoid::MessageView::mainInstance ()->putln ("Roboptim Retargeting - launching thread");
    thread_ = boost::make_shared<boost::thread>
      (boost::bind (&RoboptimRetargetingPlugin::buildProblemAndOptimize,
		    this, body, character, markerMotionItem));

    if (menuItem_ && thread_)
      menuItem_->setEnabled (false);
  }

  void buildProblemAndOptimize (cnoid::BodyPtr body,
				cnoid::CharacterPtr character,
				cnoid::MarkerMotionItemPtr markerMotionItem)
  {
    cnoid::MessageView::mainInstance ()->putln ("Roboptim Retargeting - start");

    log4cxx::LoggerPtr logger
      (log4cxx::Logger::getLogger
       ("roboptim.retargeting.choreonoid"));

    bool enableBoneLength = true;
    bool enablePosition = false;
    bool enableCollision = false;
    bool enableTorque = false;
    bool enableZmp = false;
    std::string solverName = "ipopt-sparse";

    // Retarget motion.
    roboptim::retargeting::Retarget retarget
      (markerMotionItem->motion (),
       character,
       body,
       enableBoneLength,
       enablePosition,
       enableCollision,
       enableTorque,
       enableZmp,
       solverName);

    retarget.solve ();

    cnoid::MarkerMotionItemPtr resultItem =
      boost::dynamic_pointer_cast<cnoid::MarkerMotionItem>
      (markerMotionItem->duplicate ());

    resultItem->setName("roboptim-retargeting-result");

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
	x = result.lastState ()->x;

	std::stringstream ss;
	ss << "No solution has been found. Failing..."
	   << std::endl
	   << boost::get<roboptim::SolverError>
	  (retarget.result ()).what ();
	cnoid::MessageView::mainInstance ()->putln (ss.str ());
      }
    else if (retarget.result ().which () ==
	roboptim::retargeting::Retarget::solver_t::SOLVER_VALUE_WARNINGS)
      {
	const roboptim::Result& result =
	  boost::get<roboptim::ResultWithWarnings> (retarget.result ());
	x = result.x;
	LOG4CXX_WARN (logger, "solver warnings: " << result);
      }
    else if (retarget.result ().which () ==
	     roboptim::retargeting::Retarget::solver_t::SOLVER_VALUE)
      {
	const roboptim::Result& result =
	  boost::get<roboptim::Result> (retarget.result ());
	x = result.x;
	LOG4CXX_WARN (logger, "result: " << result);
      }
    else
      {
	cnoid::MessageView::mainInstance ()->putln ("Roboptim Retargeting - failed");
	if (menuItem_)
	  menuItem_->setEnabled (true);
	thread_.reset (); // yeah suicide time.
	return;
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
	x.segment<3>
	(frameId * retarget.animatedMesh ()->optimizationVectorSizeOneFrame ()
	 + markerId * 3);

    cnoid::callSynchronously
      (boost::bind (&cnoid::RootItem::addChildItem,
		    cnoid::ItemTreeView::instance()->rootItem (),
		    resultItem,
		    false));

    cnoid::MessageView::mainInstance ()->putln ("Roboptim Retargeting - end");
    if (menuItem_)
      menuItem_->setEnabled (true);
    thread_.reset (); // yeah suicide time.
  }

  cnoid::Action* menuItem_;
  boost::shared_ptr<boost::thread> thread_;
};

CNOID_IMPLEMENT_PLUGIN_ENTRY (RoboptimRetargetingPlugin)
