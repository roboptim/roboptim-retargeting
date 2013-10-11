#include <sstream>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <QBoxLayout>
#include <QDialogButtonBox>
#include <QLabel>
#include <QMessageBox>

#include <roboptim/retargeting/problem/minimum-jerk.hh>

#include <cnoid/Archive>
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/Button>
#include <cnoid/ComboBox>
#include <cnoid/Dialog>
#include <cnoid/FolderItem>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>
#include <cnoid/LazyCaller>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/Plugin>
#include <cnoid/RootItem>
#include <cnoid/SpinBox>

#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>

#include "directories.hh"

#define _(x) x

class MinimumJerkPlugin;

class MinimumJerkDialog : public cnoid::Dialog
{
public:
  MinimumJerkDialog (MinimumJerkPlugin& plugin)
    : cnoid::Dialog (),
      plugin_ (plugin),
      solver_ (),
      nFrames_ (),
      dt_ (),
      enableFreeze_ (),
      enableVelocity_ (),
      enablePosition_ (),
      enableTorque_ (),
      enableZmp_ ()
  {
    setWindowTitle(_ ("Setup Minimum Jerk Problem"));

    QVBoxLayout* vbox = new QVBoxLayout ();
    setLayout (vbox);

    QHBoxLayout* hbox = new QHBoxLayout ();
    hbox->addSpacing (40);
    hbox->addWidget (new QLabel (_ ("Solver")));
    solver_.addItem ("cfsqp");
    solver_.addItem ("ipopt");
    solver_.addItem ("nag");
    solver_.setEditable (true);
    solver_.setCurrentIndex (1);
    hbox->addWidget (&solver_);
    hbox->addStretch ();
    vbox->addLayout (hbox);

    hbox = new QHBoxLayout ();
    hbox->addSpacing (40);
    hbox->addWidget (new QLabel (_ ("Number of frames")));
    nFrames_.setDecimals (0);
    nFrames_.setRange (10, 1000);
    nFrames_.setValue (20);
    hbox->addWidget (&nFrames_);
    hbox->addStretch ();
    vbox->addLayout (hbox);

    hbox = new QHBoxLayout ();
    hbox->addSpacing (40);
    hbox->addWidget(new QLabel (_ ("Time discretization (dt)")));
    dt_.setDecimals (3);
    dt_.setRange (0.001, 1.);
    dt_.setValue (0.1);
    hbox->addWidget(&dt_);
    hbox->addStretch ();
    vbox->addLayout (hbox);

    hbox = new QHBoxLayout ();
    enableFreeze_.setText (_ ("Freeze first frame"));
    enableFreeze_.setChecked (true);
    hbox->addWidget (&enableFreeze_);
    hbox->addStretch ();
    vbox->addLayout (hbox);

    hbox = new QHBoxLayout ();
    enableVelocity_.setText (_ ("Joint velocity"));
    enableVelocity_.setChecked (true);
    hbox->addWidget (&enableVelocity_);
    hbox->addStretch ();
    vbox->addLayout (hbox);

    hbox = new QHBoxLayout ();
    enablePosition_.setText (_ ("Feet position"));
    enablePosition_.setChecked (true);
    hbox->addWidget (&enablePosition_);
    hbox->addStretch ();
    vbox->addLayout (hbox);

    hbox = new QHBoxLayout ();
    enableTorque_.setText (_ ("Torque"));
    enableTorque_.setChecked (false);
    hbox->addWidget (&enableTorque_);
    hbox->addStretch ();
    vbox->addLayout (hbox);

    hbox = new QHBoxLayout ();
    enableZmp_.setText (_ ("ZMP"));
    enableZmp_.setChecked (true);
    hbox->addWidget (&enableZmp_);
    hbox->addStretch ();
    vbox->addLayout (hbox);

    cnoid::PushButton* applyButton = new cnoid::PushButton (_ ("&Solve"));
    applyButton->setDefault (true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox (this);
    buttonBox->addButton (applyButton, QDialogButtonBox::AcceptRole);
    applyButton->sigClicked ().connect
      (boost::bind (&MinimumJerkDialog::accept, this));

    vbox->addWidget(buttonBox);
  }

  bool store (cnoid::Archive& archive)
  {
    //FIXME: this is probably a bad idea to store the index only.
    archive.write ("solver", solver_.currentIndex ());

    archive.write ("nFrames", nFrames_.value ());
    archive.write ("dt", dt_.value ());
    archive.write ("enableFreeze", enableFreeze_.isChecked ());
    archive.write ("enableVelocity", enableVelocity_.isChecked ());
    archive.write ("enablePosition", enablePosition_.isChecked ());
    archive.write ("enableTorque", enableTorque_.isChecked ());
    archive.write ("enableZmp", enableZmp_.isChecked ());
    return true;
  }
  void restore (const cnoid::Archive& archive)
  {
    //FIXME: this is probably a bad idea to store the index only.
    solver_.setCurrentIndex
      (archive.get ("solver", solver_.currentIndex ()));

    nFrames_.setValue
      (archive.get ("nFrames", nFrames_.value ()));
    dt_.setValue (archive.get ("dt", dt_.value ()));
    enableFreeze_.setChecked
      (archive.get ("enableFreeze", enableFreeze_.isChecked ()));
    enableVelocity_.setChecked
      (archive.get ("enableVelocity", enableVelocity_.isChecked ()));
    enablePosition_.setChecked
      (archive.get ("enablePosition", enablePosition_.isChecked ()));
    enableTorque_.setChecked
      (archive.get ("enableTorque", enableTorque_.isChecked ()));
    enableZmp_.setChecked
      (archive.get ("enableZmp", enableZmp_.isChecked ()));
  }

  virtual void onAccepted ();

public:
  MinimumJerkPlugin& plugin_;

  cnoid::ComboBox solver_;

  cnoid::DoubleSpinBox nFrames_;
  cnoid::DoubleSpinBox dt_;

  cnoid::CheckBox enableFreeze_;
  cnoid::CheckBox enableVelocity_;
  cnoid::CheckBox enablePosition_;
  cnoid::CheckBox enableTorque_;
  cnoid::CheckBox enableZmp_;
};

class MinimumJerkPlugin : public cnoid::Plugin
{
public:
  typedef roboptim::retargeting::problem::MinimumJerk::size_type size_type;
  typedef roboptim::retargeting::problem::MinimumJerk::solver_t solver_t;

  explicit MinimumJerkPlugin ()
    : Plugin("RoboptimMinimumJerk"),
      menuItem_ (),
      menuItemSolve_ (),
      thread_ (),
      errorBox_
      (QMessageBox::Critical,
       "Error while solving minimum jerk problem",
       "An exception has been thrown during problem optimization.",
       QMessageBox::Ok),
      rootItem_ (cnoid::ItemTreeView::instance()->rootItem ()),
      folderIteration_ (),
      dialog_ ()
  {
    errorBox_.setWindowFlags
      (errorBox_.windowFlags () & ~Qt::WindowCloseButtonHint);
  }

  virtual bool initialize ()
  {
    // Initialize logging system.
    std::string log4cxxConfigurationFile = PKG_SHARE_DIR;
    log4cxxConfigurationFile += "/log4cxx.xml";
    log4cxx::xml::DOMConfigurator::configure (log4cxxConfigurationFile);

    if (!dialog_)
      {
	dialog_ = this->manage (new MinimumJerkDialog (*this));
	menuItem_ =
	  menuManager ().setPath ("/Filters").addItem ("Roboptim Minimum Jerk (setup and solve)");
	menuItem_->sigTriggered ().connect
	  (boost::bind (&MinimumJerkPlugin::menuCb, this));
	menuItemSolve_ =
	  menuManager ().setPath ("/Filters").addItem ("Roboptim Minimum Jerk (solve)");
	menuItemSolve_->sigTriggered ().connect
	  (boost::bind (&MinimumJerkPlugin::solve, this));

        connectProjectArchiver
	  ("MinimumJerkDialog",
	   bind(&MinimumJerkDialog::store, dialog_, _1),
	   bind(&MinimumJerkDialog::restore, dialog_, _1));
      }

    return true;
  }

  void solve ()
  {
    if (thread_)
      {
	cnoid::MessageView::mainInstance ()->putln
	  ("Roboptim Minimum Jerk - thread already launched, ignoring command");
	return;
      }

    cnoid::BodyPtr body;
    cnoid::BodyItemPtr bodyItem;
    cnoid::BodyMotionItemPtr bodyMotionItem;
    cnoid::ItemList<cnoid::Item> selected =
      cnoid::ItemTreeView::instance()->selectedItems<cnoid::Item>();
    cnoid::ItemList<cnoid::BodyItem> bodyItems = selected;
    cnoid::ItemList<cnoid::BodyMotionItem> bodyMotionItems = selected;

    for (std::size_t i = 0; !body && i < bodyItems.size (); ++i)
      {
	int i_ = static_cast<int> (i);
	cnoid::BodyItemPtr bodyItemTmp = bodyItems.get (i_);
	if (bodyItemTmp->body ())
	  {
	    body = bodyItemTmp->body ();
	    bodyItem = bodyItemTmp;
	  }
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
	QMessageBox::critical
	  (0,
	   "Error while initializing minimum jerk optimization",
	   "Please, select a BodyMotion item (anything will do) "
	   "before trying to solve the problem.");
	return;
      }
    if (!body && !bodyItem)
      {
	QMessageBox::critical
	  (0,
	   "Error while initializing minimum jerk optimization",
	   "Please, select the robot you want to use in the optimization process "
	   "before trying to solve the problem.");
	return;
      }

    // Create folder
    static int callId = 0;
    cnoid::FolderItem* folder = new cnoid::FolderItem ();
    if (callId == 0)
      folder->setName ("RobOptim Minimum Jerk");
    else
      folder->setName
	((boost::format ("RobOptim Minimum Jerk (%d)") % callId).str ());
    ++callId;
    folderIteration_ = new cnoid::FolderItem ();
    folderIteration_->setName ("Intermediate iterations");
    folder->addChildItem (folderIteration_);

    bodyItem->addChildItem (folder);
    rootItem_ = folder;

    // Launch thread
    cnoid::MessageView::mainInstance ()->putln
      ("Roboptim Minimum Jerk - launching thread");
    thread_ = boost::make_shared<boost::thread>
      (boost::bind
       (&MinimumJerkPlugin::buildProblemAndOptimize,
	this, body, bodyMotionItem));

    if (thread_)
      {
	if (menuItem_)
	  menuItem_->setEnabled (false);
	if (menuItemSolve_)
	  menuItemSolve_->setEnabled (false);
      }
  }

private:
  void menuCb ()
  {
    if (dialog_)
      dialog_->show ();
  }

  void perIterationCallback
  (roboptim::retargeting::problem::MinimumJerk& minimumJerkProblem,
   cnoid::BodyMotionItemPtr bodyMotionItem,
   const solver_t::vector_t& x,
   const solver_t::problem_t&)
  {
    static int iteration = 0;
    boost::format name ("iteration %d");
    name % iteration;
    if (folderIteration_)
      addResultToTree
	(minimumJerkProblem,
	 bodyMotionItem, name.str (), x, folderIteration_);
    else
      addResultToTree
	(minimumJerkProblem,
	 bodyMotionItem, name.str (), x);
    ++iteration;
  }

  void addResultToTree
  (roboptim::retargeting::problem::MinimumJerk& minimumJerkProblem,
   cnoid::BodyMotionItemPtr bodyMotionItem,
   const std::string& name,
   const solver_t::vector_t& x,
   cnoid::ItemPtr parentItem)
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
      (boost::bind
       (&cnoid::Item::addChildItem, parentItem, resultItem, false));

  }

  void addResultToTree
  (roboptim::retargeting::problem::MinimumJerk& minimumJerkProblem,
   cnoid::BodyMotionItemPtr bodyMotionItem,
   const std::string& name,
   const solver_t::vector_t& x)
  {
    addResultToTree
      (minimumJerkProblem, bodyMotionItem, name, x, rootItem_);
  }

  void buildProblemAndOptimize (cnoid::BodyPtr robot,
				cnoid::BodyMotionItemPtr bodyMotionItem)
  {
    cnoid::MessageView::mainInstance ()->putln
      ("Roboptim Minimum Jerk - start");

    log4cxx::LoggerPtr logger
      (log4cxx::Logger::getLogger
       ("roboptim.retargeting.choreonoid"));

    if (!dialog_)
      throw std::runtime_error ("dialog box is missing");

    // Retrieve problem setup from dialog window.
    // The dialog window values are restored from the loaded project.
    unsigned nFrames = dialog_->nFrames_.value ();
    double dt = dialog_->dt_.value ();
    bool enableFreeze = dialog_->enableFreeze_.isChecked ();
    bool enableVelocity = dialog_->enableVelocity_.isChecked ();
    bool enablePosition = dialog_->enablePosition_.isChecked ();
    bool enableCollision = false;
    bool enableTorque = dialog_->enableTorque_.isChecked ();
    bool enableZmp = dialog_->enableZmp_.isChecked ();
    std::string solverName =
      dialog_->solver_.currentText ().toUtf8 ().constData ();

    // Build optimization problem.
    roboptim::retargeting::problem::MinimumJerk
      minimumJerkProblem
      (robot, nFrames, dt, enableFreeze, enableVelocity,
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
	 "initial trajectory",
	 *(minimumJerkProblem.problem ()->startingPoint ()));

    try
      {
	minimumJerkProblem.solve ();
      }
    catch (std::runtime_error& e)
      {
	cnoid::callSynchronously
	  (boost::bind
	   (&QMessageBox::setDetailedText, boost::ref (errorBox_), e.what ()));
	cnoid::callSynchronously
	  (boost::bind (&QMessageBox::exec, boost::ref (errorBox_)));
	cnoid::MessageView::mainInstance ()->putln (e.what ());
	if (menuItem_)
	  menuItem_->setEnabled (true);
	if (menuItemSolve_)
	  menuItemSolve_->setEnabled (true);
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
	if (menuItemSolve_)
	  menuItemSolve_->setEnabled (true);
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
	if (menuItemSolve_)
	  menuItemSolve_->setEnabled (true);
	thread_.reset (); // yeah suicide time.
	return;
      }

    addResultToTree
      (minimumJerkProblem, bodyMotionItem, "final result", x);
    cnoid::MessageView::mainInstance ()->putln ("Roboptim Minimum Jerk - end");
    if (menuItem_)
      menuItem_->setEnabled (true);
    if (menuItemSolve_)
      menuItemSolve_->setEnabled (true);
    thread_.reset (); // yeah suicide time.
  }

  cnoid::Action* menuItem_;
  cnoid::Action* menuItemSolve_;
  boost::shared_ptr<boost::thread> thread_;

  QMessageBox errorBox_;

  cnoid::ItemPtr rootItem_;
  cnoid::FolderItem* folderIteration_;

  MinimumJerkDialog* dialog_;
};

CNOID_IMPLEMENT_PLUGIN_ENTRY (MinimumJerkPlugin)

void
MinimumJerkDialog::onAccepted ()
{
  plugin_.solve ();
}
