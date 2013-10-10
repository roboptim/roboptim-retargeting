#include <sstream>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <roboptim/retargeting/problem/minimum-jerk.hh>

#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>
#include <cnoid/LazyCaller>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/Plugin>
#include <cnoid/RootItem>

#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>

#include "directories.hh"

class MinimumJerkPlugin : public cnoid::Plugin
{
public:
  typedef roboptim::retargeting::problem::MinimumJerk::size_type size_type;
  typedef roboptim::retargeting::problem::MinimumJerk::solver_t solver_t;

  explicit MinimumJerkPlugin ()
    : Plugin("RoboptimMinimumJerk"),
      menuItem_ (),
      thread_ ()
  {}

  virtual bool initialize ()
  {
    // Initialize logging system.
    std::string log4cxxConfigurationFile = PKG_SHARE_DIR;
    log4cxxConfigurationFile += "/log4cxx.xml";
    log4cxx::xml::DOMConfigurator::configure (log4cxxConfigurationFile);

    menuItem_ =
      menuManager ().setPath ("/Filters").addItem ("Roboptim Minimum Jerk");
    menuItem_->sigTriggered ().connect
      (boost::bind (&MinimumJerkPlugin::menuCb, this));
    return true;
  }

private:
  void menuCb ()
  {
    if (thread_)
      {
	cnoid::MessageView::mainInstance ()->putln
	  ("Roboptim Minimum Jerk - thread already launched, ignoring command");
	return;
      }

    cnoid::BodyPtr body;
    cnoid::BodyMotionItemPtr bodyMotionItem;
    cnoid::ItemList<cnoid::Item> selected =
      cnoid::ItemTreeView::instance()->selectedItems<cnoid::Item>();
    cnoid::ItemList<cnoid::BodyItem> bodyItems = selected;
    cnoid::ItemList<cnoid::BodyMotionItem> bodyMotionItems = selected;

    for (std::size_t i = 0; i < bodyItems.size (); ++i)
      {
	int i_ = static_cast<int> (i);
	cnoid::BodyItem* bodyItem = bodyItems.get (i_);
	if (bodyItem->body ())
	  body = bodyItem->body ();
      }

    for (std::size_t i = 0; i < bodyMotionItems.size (); ++i)
      {
	int i_ = static_cast<int> (i);
	cnoid::BodyMotionItemPtr bodyMotionItemTmp = bodyMotionItems.get (i_);
	if (bodyMotionItemTmp)
	  bodyMotionItem = bodyMotionItemTmp;
      }

    if (!bodyMotionItem)
      {
	cnoid::MessageView::mainInstance ()->putln
	  ("no robot motion item is selected");
	return;
      }
    if (!body)
      {
	cnoid::MessageView::mainInstance ()->putln
	  ("no robot model item is selected");
	return;
      }

    cnoid::MessageView::mainInstance ()->putln
      ("Roboptim Minimum Jerk - launching thread");
    thread_ = boost::make_shared<boost::thread>
      (boost::bind (&MinimumJerkPlugin::buildProblemAndOptimize,
		    this, body, bodyMotionItem));

    if (menuItem_ && thread_)
      menuItem_->setEnabled (false);
  }

  void perIterationCallback
  (roboptim::retargeting::problem::MinimumJerk& minimumJerkProblem,
   cnoid::BodyMotionItemPtr bodyMotionItem,
   const solver_t::vector_t& x,
   const solver_t::problem_t&)
  {
    static int iteration = 0;
    boost::format name ("minimum-jerk - iteration %d");
    name % iteration;
    addResultToTree (minimumJerkProblem, bodyMotionItem, name.str (), x);
    ++iteration;
  }

  void addResultToTree
  (roboptim::retargeting::problem::MinimumJerk& minimumJerkProblem,
   cnoid::BodyMotionItemPtr bodyMotionItem,
   const std::string& name,
   const solver_t::vector_t& x)
  {
    cnoid::BodyMotionItemPtr resultItem =
      boost::dynamic_pointer_cast<cnoid::BodyMotionItem>
      (bodyMotionItem->duplicate ());

    resultItem->setName (name);
    resultItem->motion ()->setDimension
      (minimumJerkProblem.nFrames (), minimumJerkProblem.nDofs () - 6);
    resultItem->motion ()->setFrameRate (1. / minimumJerkProblem.dt ());

    // Copy joint values.
    for (int frameId = 0; frameId < minimumJerkProblem.nFrames (); ++frameId)
      for (std::size_t jointId = 0; jointId < minimumJerkProblem.nDofs () - 6;
    	   ++jointId)
    	resultItem->jointPosSeqItem ()->seq ()->frame (frameId)[jointId] =
    	  x[frameId * minimumJerkProblem.nDofs () + 6 + jointId];

    // Copy base link positions and attitudes.
    for (int frameId = 0; frameId < minimumJerkProblem.nFrames (); ++frameId)
      for (std::size_t jointId = 0; jointId < minimumJerkProblem.nDofs () - 6;
    	   ++jointId)
    	{
    	  resultItem->linkPosSeqItem ()->seq ()->frame
    	    (frameId)[jointId].translation () =
    	    x.segment(frameId * minimumJerkProblem.nDofs (), 3);

    	  cnoid::Vector3 axis =
    	    x.segment(frameId * minimumJerkProblem.nDofs () + 3, 3);
    	  double angle = axis.norm ();

	  // If axis norm is null, set to identity manually.
	  if (angle >= 1e-8)
	    {
	      axis.normalize ();
	      Eigen::AngleAxisd angleAxis (angle, axis);
	      Eigen::Quaterniond q (angleAxis);
	      resultItem->linkPosSeqItem ()->seq ()->frame
		(frameId)[jointId].rotation () = q;
	    }
	  else
	    {
	      resultItem->linkPosSeqItem ()->seq ()->frame
		(frameId)[jointId].rotation () = Eigen::Quaterniond ();
	    }
    	}

    cnoid::callSynchronously
      (boost::bind (&cnoid::RootItem::addChildItem,
		    cnoid::ItemTreeView::instance()->rootItem (),
		    resultItem,
		    false));

  }

  void buildProblemAndOptimize (cnoid::BodyPtr robot,
				cnoid::BodyMotionItemPtr bodyMotionItem)
  {
    cnoid::MessageView::mainInstance ()->putln
      ("Roboptim Minimum Jerk - start");

    log4cxx::LoggerPtr logger
      (log4cxx::Logger::getLogger
       ("roboptim.retargeting.choreonoid"));

    bool enableFreeze = true;
    bool enableVelocity = true;
    bool enablePosition = true;
    bool enableCollision = false;
    bool enableTorque = false;
    bool enableZmp = true;
    std::string solverName = "cfsqp";

    // Build optimization problem.
    roboptim::retargeting::problem::MinimumJerk
      minimumJerkProblem
      (robot, enableFreeze, enableVelocity,
       enablePosition, enableCollision,
       enableTorque, enableZmp, solverName);

    solver_t::callback_t cb = boost::bind
      (&MinimumJerkPlugin::perIterationCallback,
       this, minimumJerkProblem, bodyMotionItem, _1, _2);
    minimumJerkProblem.additionalCallback () = cb;

    if (minimumJerkProblem.problem ()->startingPoint ())
      addResultToTree
	(minimumJerkProblem,
	 bodyMotionItem,
	 "minimum-jerk - initial trajectory",
	 *(minimumJerkProblem.problem ()->startingPoint ()));

    try
      {
	minimumJerkProblem.solve ();
      }
    catch (std::runtime_error& e)
      {
	cnoid::MessageView::mainInstance ()->putln (e.what ());
	if (menuItem_)
	  menuItem_->setEnabled (true);
	thread_.reset ();
	return;
      }

    // Get the result.
    Eigen::VectorXd x;

    // Populate result.
    // Check if the minimization has succeed.
    if (minimumJerkProblem.result ().which () == solver_t::SOLVER_ERROR)
      {
	const roboptim::SolverError& result =
	  boost::get<roboptim::SolverError> (minimumJerkProblem.result ());
	if (result.lastState ())
	  x = result.lastState ()->x;
	else
	  x.setZero ();

	std::stringstream ss;
	ss << "No solution has been found. Failing..."
	   << std::endl
	   << boost::get<roboptim::SolverError>
	  (minimumJerkProblem.result ()).what ();
	cnoid::MessageView::mainInstance ()->putln (ss.str ());
      }
    else if (minimumJerkProblem.result ().which () ==
	     solver_t::SOLVER_VALUE_WARNINGS)
      {
	const roboptim::Result& result =
	  boost::get<roboptim::ResultWithWarnings>
	  (minimumJerkProblem.result ());
	x = result.x;
	LOG4CXX_WARN (logger, "solver warnings: " << result);
      }
    else if (minimumJerkProblem.result ().which () == solver_t::SOLVER_VALUE)
      {
	const roboptim::Result& result =
	  boost::get<roboptim::Result> (minimumJerkProblem.result ());
	x = result.x;
	LOG4CXX_WARN (logger, "result: " << result);
      }
    else
      {
	cnoid::MessageView::mainInstance
	  ()->putln ("Roboptim Minimum Jerk - failed");
	if (menuItem_)
	  menuItem_->setEnabled (true);
	thread_.reset (); // yeah suicide time.
	return;
      }

    // It failed, do not add the final result to choreonoid.
    if (x.size () == 0)
      {
	cnoid::MessageView::mainInstance
	  ()->putln ("Roboptim Minimum Jerk - end");
	if (menuItem_)
	  menuItem_->setEnabled (true);
	thread_.reset (); // yeah suicide time.
	return;
      }

    addResultToTree
      (minimumJerkProblem, bodyMotionItem,
       "minimum-jerk - final result", x);
    cnoid::MessageView::mainInstance ()->putln ("Roboptim Minimum Jerk - end");
    if (menuItem_)
      menuItem_->setEnabled (true);
    thread_.reset (); // yeah suicide time.
  }

  cnoid::Action* menuItem_;
  boost::shared_ptr<boost::thread> thread_;
};

CNOID_IMPLEMENT_PLUGIN_ENTRY (MinimumJerkPlugin)
