#include <algorithm>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <QBoxLayout>
#include <QDialogButtonBox>
#include <QGroupBox>
#include <QLabel>
#include <QMessageBox>
#include <QScrollArea>
#include <QTabWidget>

#include <roboptim/retargeting/problem/joint.hh>

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

class JointPlugin;

class JointMainDialog : public QWidget
{
public:
  explicit JointMainDialog (QWidget* parent = 0)
    : QWidget (parent)
  {
    QVBoxLayout* vbox = new QVBoxLayout;

    QHBoxLayout* hbox = new QHBoxLayout;
    hbox->addWidget (new QLabel (_ ("Solver")));
    solver_.addItem ("cfsqp");
    solver_.addItem ("ipopt");
    solver_.addItem ("nag-nlp");
    solver_.setEditable (true);
    solver_.setCurrentIndex (1);
    hbox->addWidget (&solver_);
    hbox->addStretch ();
    vbox->addLayout (hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget (new QLabel (_ ("Trajectory Type")));
    trajectoryType_.addItem ("interpolation");
    trajectoryType_.addItem ("cubic-spline");
    trajectoryType_.setEditable (false);
    trajectoryType_.setCurrentIndex (1);
    hbox->addWidget (&trajectoryType_);
    hbox->addStretch ();
    vbox->addLayout (hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel (_ ("Number of nodes (spline only)")));
    nNodes_.setDecimals (0);
    nNodes_.setRange (5, 10000);
    nNodes_.setValue (10);
    hbox->addWidget(&nNodes_);
    hbox->addStretch ();
    vbox->addLayout (hbox);

    QGroupBox* constraints = new QGroupBox ("Constraints");

    QVBoxLayout* vboxConstraints = new QVBoxLayout;
    constraints->setLayout (vboxConstraints);

    hbox = new QHBoxLayout;
    enableFreeze_.setText (_ ("Freeze first frame"));
    enableFreeze_.setChecked (true);
    hbox->addWidget (&enableFreeze_);
    hbox->addStretch ();
    vboxConstraints->addLayout (hbox);

    hbox = new QHBoxLayout;
    enableVelocity_.setText (_ ("Joint velocity"));
    enableVelocity_.setChecked (true);
    hbox->addWidget (&enableVelocity_);
    hbox->addStretch ();
    vboxConstraints->addLayout (hbox);

    hbox = new QHBoxLayout;
    enablePosition_.setText (_ ("Feet position"));
    enablePosition_.setChecked (true);
    hbox->addWidget (&enablePosition_);
    hbox->addStretch ();
    vboxConstraints->addLayout (hbox);

    hbox = new QHBoxLayout;
    enableTorque_.setText (_ ("Torque"));
    enableTorque_.setChecked (false);
    hbox->addWidget (&enableTorque_);
    hbox->addStretch ();
    vboxConstraints->addLayout (hbox);

    hbox = new QHBoxLayout;
    enableZmp_.setText (_ ("ZMP"));
    enableZmp_.setChecked (true);
    hbox->addWidget (&enableZmp_);
    hbox->addStretch ();
    vboxConstraints->addLayout (hbox);

    vbox->addWidget (constraints);

    vbox->addStretch (1);
    setLayout (vbox);
  }

  cnoid::ComboBox& solver ()
  {
    return solver_;
  }

  cnoid::ComboBox& trajectoryType ()
  {
    return trajectoryType_;
  }

  cnoid::DoubleSpinBox& nNodes ()
  {
    return nNodes_;
  }


  cnoid::CheckBox& enableFreeze ()
  {
    return enableFreeze_;
  }
  cnoid::CheckBox& enableVelocity ()
  {
    return enableVelocity_;
  }
  cnoid::CheckBox& enablePosition ()
  {
    return enablePosition_;
  }
  cnoid::CheckBox& enableTorque ()
  {
    return enableTorque_;
  }
  cnoid::CheckBox& enableZmp ()
  {
    return enableZmp_;
  }

private:
  cnoid::ComboBox solver_;

  cnoid::ComboBox trajectoryType_;

  /// \brief Number of nodes when spline is used
  cnoid::DoubleSpinBox nNodes_;

  /// \name Constraints
  /// \{
  cnoid::CheckBox enableFreeze_;
  cnoid::CheckBox enableVelocity_;
  cnoid::CheckBox enablePosition_;
  cnoid::CheckBox enableTorque_;
  cnoid::CheckBox enableZmp_;
  /// \}
};

class JointDofDialog : public QWidget
{
public:
  explicit JointDofDialog (QWidget* parent = 0)
    : QWidget (parent)
  {
    //FIXME: for now only HRP-4C is supported.
    boost::array<QString, 48> dofs = {{
	"Free floating X",
	"Free floating Y",
	"Free floating Z",
	"Free floating angle axis alpha",
	"Free floating angle axis beta",
	"Free floating angle axis gamma",
	"R_HIP_Y",
	"R_HIP_R",
	"R_HIP_P",
	"R_KNEE_P",
	"R_ANKLE_P",
	"R_ANKLE_R",
	"L_HIP_Y",
	"L_HIP_R",
	"L_HIP_P",
	"L_KNEE_P",
	"L_ANKLE_P",
	"L_ANKLE_R",
	"CHEST_P",
	"CHEST_R",
	"CHEST_Y",
	"NECK_Y",
	"NECK_R",
	"NECK_P",
	"EYEBROW_P",
	"EYELID_P",
	"EYE_P",
	"EYE_Y",
	"MOUTH_P",
	"LOWERLIP_P",
	"UPPERLIP_P",
	"CHEEK_P",
	"R_SHOULDER_P",
	"R_SHOULDER_R",
	"R_SHOULDER_Y",
	"R_ELBOW_P",
	"R_WRIST_Y",
	"R_WRIST_R",
	"R_HAND_J0",
	"R_HAND_J1",
	"L_SHOULDER_P",
	"L_SHOULDER_R",
	"L_SHOULDER_Y",
	"L_ELBOW_P",
	"L_WRIST_Y",
	"L_WRIST_R",
	"L_HAND_J0",
	"L_HAND_J1"
      }};

    QGroupBox* dofGroup = new QGroupBox (_ ("Degrees of Freedom to be considered"));
    QVBoxLayout* dofLayout = new QVBoxLayout;

    for (std::size_t dofId = 0; dofId < dofs.size (); ++dofId)
      {
	QCheckBox* isDofActive = new QCheckBox (dofs[dofId]);
	isDofActive->setChecked (true);
	dofLayout->addWidget (isDofActive);
	dofs_.push_back (isDofActive);
      }
    dofGroup->setLayout (dofLayout);

    QScrollArea* scrollarea = new QScrollArea;
    scrollarea->setHorizontalScrollBarPolicy (Qt::ScrollBarAlwaysOff);
    scrollarea->setVerticalScrollBarPolicy (Qt::ScrollBarAsNeeded);
    scrollarea->setWidget (dofGroup);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget (scrollarea);
    mainLayout->addStretch (1);
    setLayout (mainLayout);
  }

  const std::vector<QCheckBox*>& dofs () const throw ()
  {
    return dofs_;
  }

private:
  std::vector<QCheckBox*> dofs_;
};

class JointDialog : public cnoid::Dialog
{
public:
  JointDialog (JointPlugin& plugin)
    : cnoid::Dialog (),
      plugin_ (plugin),
      generalTab_ (new JointMainDialog ()),
      dofTab_ (new JointDofDialog ())
  {
    setWindowTitle(_ ("Setup Joint Optimization Problem"));

    QVBoxLayout* vbox = new QVBoxLayout ();

    QTabWidget* tabs = new QTabWidget ();
    tabs->addTab (generalTab_, "General");
    tabs->addTab (dofTab_, "Degrees of Freedom");
    vbox->addWidget (tabs);

    cnoid::PushButton* applyButton = new cnoid::PushButton (_ ("&Solve"));
    applyButton->setDefault (true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox (this);
    buttonBox->addButton (applyButton, QDialogButtonBox::AcceptRole);
    applyButton->sigClicked ().connect
      (boost::bind (&JointDialog::accept, this));
    vbox->addWidget(buttonBox);

    vbox->addStretch (1);
    setLayout (vbox);
  }

  JointMainDialog* generalTab ()
  {
    return generalTab_;
  }
  JointDofDialog* dofTab ()
  {
    return dofTab_;
  }


  bool store (cnoid::Archive& archive)
  {
    //FIXME: this is probably a bad idea to store the index only.
    archive.write ("solver", generalTab ()->solver ().currentIndex ());

    //FIXME: this is probably a bad idea to store the index only.
    archive.write ("trajectoryType",
		   generalTab ()->trajectoryType ().currentIndex ());

    archive.write ("enableFreeze", generalTab ()->enableFreeze ().isChecked ());
    archive.write ("enableVelocity",
		   generalTab ()->enableVelocity ().isChecked ());
    archive.write ("enablePosition",
		   generalTab ()->enablePosition ().isChecked ());
    archive.write ("enableTorque", generalTab ()->enableTorque ().isChecked ());
    archive.write ("enableZmp", generalTab ()->enableZmp ().isChecked ());

    for (std::size_t dofId = 0; dofId < dofTab ()->dofs ().size (); ++dofId)
      archive.write
	((boost::format ("dof[%d]") % dofId).str (),
	 dofTab ()->dofs ()[dofId]->isChecked ());

    return true;
  }
  void restore (const cnoid::Archive& archive)
  {
    //FIXME: this is probably a bad idea to store the index only.
    generalTab ()->solver ().setCurrentIndex
      (archive.get ("solver",
		    generalTab ()->solver ().currentIndex ()));

    //FIXME: this is probably a bad idea to store the index only.
    generalTab ()->solver ().setCurrentIndex
      (archive.get ("trajectoryType",
		    generalTab ()->trajectoryType ().currentIndex ()));

    generalTab ()->enableFreeze ().setChecked
      (archive.get ("enableFreeze",
		    generalTab ()->enableFreeze ().isChecked ()));
    generalTab ()->enableVelocity ().setChecked
      (archive.get ("enableVelocity",
		    generalTab ()->enableVelocity ().isChecked ()));
    generalTab ()->enablePosition ().setChecked
      (archive.get ("enablePosition",
		    generalTab ()->enablePosition ().isChecked ()));
    generalTab ()->enableTorque ().setChecked
      (archive.get ("enableTorque",
		    generalTab ()->enableTorque ().isChecked ()));
    generalTab ()->enableZmp ().setChecked
      (archive.get ("enableZmp",
		    generalTab ()->enableZmp ().isChecked ()));

    for (std::size_t dofId = 0; dofId < dofTab ()->dofs ().size (); ++dofId)
      dofTab ()->dofs ()[dofId]->setChecked
	(archive.get
	 ((boost::format ("dof[%d]") % dofId).str (),
	  dofTab ()->dofs ()[dofId]->isChecked ()));
  }

  virtual void onAccepted ();

public:
  JointPlugin& plugin_;

  JointMainDialog* generalTab_;
  JointDofDialog* dofTab_;
};

class JointPlugin : public cnoid::Plugin
{
public:
  typedef roboptim::retargeting::problem::Joint::size_type size_type;
  typedef roboptim::retargeting::problem::Joint::solver_t solver_t;

  explicit JointPlugin ()
    : Plugin("RoboptimJoint"),
      menuItem_ (),
      menuItemSolve_ (),
      thread_ (),
      errorBox_
      (QMessageBox::Critical,
       "Error while solving minimum jerk problem",
       "An exception has been thrown during problem optimization.",
       QMessageBox::Ok),
      nosolutionBox_
      (QMessageBox::Critical,
       "Error while solving minimum jerk problem",
       "No solution has been found during the optimization process.",
       QMessageBox::Ok),
      warningBox_
      (QMessageBox::Warning,
       "Minimum jerk problem",
       "A solution has been found but may not be totally satisfying. "
       "Please proceed carefully.",
       QMessageBox::Ok),
      finishedBox_
      (QMessageBox::Information,
       "Minimum jerk problem",
       "The optimization process finished successfully.",
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
	dialog_ = this->manage (new JointDialog (*this));
	menuItem_ =
	  menuManager ().setPath ("/Filters").addItem ("Roboptim Joint Optimization (setup and solve)");
	menuItem_->sigTriggered ().connect
	  (boost::bind (&JointPlugin::menuCb, this));
	menuItemSolve_ =
	  menuManager ().setPath ("/Filters").addItem ("Roboptim Joint Optimization (solve)");
	menuItemSolve_->sigTriggered ().connect
	  (boost::bind (&JointPlugin::solve, this));

        connectProjectArchiver
	  ("JointDialog",
	   bind(&JointDialog::store, dialog_, _1),
	   bind(&JointDialog::restore, dialog_, _1));
      }

    return true;
  }

  void solve ()
  {
    if (thread_)
      {
	cnoid::MessageView::mainInstance ()->putln
	  ("Roboptim Joint Optimization - thread already launched, ignoring command");
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
      folder->setName ("RobOptim Joint Optimization");
    else
      folder->setName
	((boost::format ("RobOptim Joint Optimization (%d)") % callId).str ());
    ++callId;
    folderIteration_ = new cnoid::FolderItem ();
    folderIteration_->setName ("Intermediate iterations");
    folder->addChildItem (folderIteration_);

    bodyItem->addChildItem (folder);
    rootItem_ = folder;

    // Launch thread
    cnoid::MessageView::mainInstance ()->putln
      ("Roboptim Joint Optimization - launching thread");
    thread_ = boost::make_shared<boost::thread>
      (boost::bind
       (&JointPlugin::buildProblemAndOptimize,
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
  (roboptim::retargeting::problem::Joint& minimumJerkProblem,
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
  (roboptim::retargeting::problem::Joint& minimumJerkProblem,
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
  (roboptim::retargeting::problem::Joint& minimumJerkProblem,
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
      ("Roboptim Joint Optimization - start");

    log4cxx::LoggerPtr logger
      (log4cxx::Logger::getLogger
       ("roboptim.retargeting.choreonoid"));

    if (!dialog_)
      throw std::runtime_error ("dialog box is missing");

    // Retrieve problem setup from dialog window.
    // The dialog window values are restored from the loaded project.
    unsigned nNodes = dialog_->generalTab ()->nNodes ().value ();
    bool enableFreeze = dialog_->generalTab ()->enableFreeze ().isChecked ();
    bool enableVelocity =
      dialog_->generalTab ()->enableVelocity ().isChecked ();
    bool enablePosition =
      dialog_->generalTab ()->enablePosition ().isChecked ();
    bool enableCollision = false;
    bool enableTorque = dialog_->generalTab ()->enableTorque ().isChecked ();
    bool enableZmp = dialog_->generalTab ()->enableZmp ().isChecked ();
    std::string solverName =
      dialog_->generalTab ()->solver ().currentText ().toUtf8 ().constData ();
    std::string trajectoryType =
      dialog_->generalTab ()->trajectoryType ()
      .currentText ().toUtf8 ().constData ();

    std::vector<bool> enabledDofs;
    for (std::size_t dofId = 0;
	 dofId < dialog_->dofTab ()->dofs ().size (); ++dofId)
      enabledDofs.push_back
	(dialog_->dofTab ()->dofs ()[dofId]->isChecked ());

    boost::array<std::string, 2> validTrajectoryType = {{
	"interpolation", "cubic-spline"
      }};
    if (std::find (validTrajectoryType.begin (), validTrajectoryType.end (),
		   trajectoryType)
	== validTrajectoryType.end ())
      {
	std::string errorMsg ("invalid trajectory type");
	cnoid::callSynchronously
	  (boost::bind
	   (&QMessageBox::setDetailedText, boost::ref (errorBox_),
	    errorMsg.c_str ()));
	cnoid::callSynchronously
	  (boost::bind (&QMessageBox::exec, boost::ref (errorBox_)));
	cnoid::MessageView::mainInstance ()->putln (errorMsg.c_str ());
	if (menuItem_)
	  menuItem_->setEnabled (true);
	if (menuItemSolve_)
	  menuItemSolve_->setEnabled (true);
	thread_.reset ();
	return;
      }

    // Build optimization problem.
    using roboptim::retargeting::problem::Joint;

    roboptim::retargeting::problem::JointShPtr_t
      minimumJerkProblem;

    if (trajectoryType == "interpolation")
      minimumJerkProblem =
	roboptim::retargeting::problem::Joint
	::buildVectorInterpolationBasedOptimizationProblem
	(robot, bodyMotionItem->motion (),
	 enableFreeze, enableVelocity,
	 enablePosition, enableCollision,
	 enableTorque, enableZmp, solverName,
	 enabledDofs);
    else if (trajectoryType == "cubic-spline")
      minimumJerkProblem =
	roboptim::retargeting::problem::Joint
	::buildSplineBasedOptimizationProblem
	(robot, bodyMotionItem->motion (), nNodes,
	 enableFreeze, enableVelocity,
	 enablePosition, enableCollision,
	 enableTorque, enableZmp, solverName,
	 enabledDofs);
    else
      throw std::runtime_error ("invalid trajectory type");


    solver_t::callback_t cb = boost::bind
      (&JointPlugin::perIterationCallback,
       this, *minimumJerkProblem, bodyMotionItem, _1, _2);
    minimumJerkProblem->additionalCallback () = cb;

    if (minimumJerkProblem->problem ()->startingPoint ())
      addResultToTree
	(*minimumJerkProblem,
	 bodyMotionItem,
	 "initial trajectory",
	 *(minimumJerkProblem->problem ()->startingPoint ()));

    try
      {
	minimumJerkProblem->solve ();
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
    if (minimumJerkProblem->result ().which () == solver_t::SOLVER_ERROR)
      {
	const roboptim::SolverError& result =
	  boost::get<roboptim::SolverError> (minimumJerkProblem->result ());
	if (result.lastState ())
	  x = result.lastState ()->x;
	else
	  x.setZero ();

	std::stringstream ss;
	ss << "No solution has been found. Failing..."
	   << std::endl
	   << boost::get<roboptim::SolverError>
	  (minimumJerkProblem->result ()).what ();

	cnoid::callSynchronously
	  (boost::bind
	   (&QMessageBox::setDetailedText,
	    boost::ref (nosolutionBox_), ss.str ().c_str ()));
	cnoid::callSynchronously
	  (boost::bind (&QMessageBox::exec, boost::ref (nosolutionBox_)));
	cnoid::MessageView::mainInstance ()->putln (ss.str ());
      }
    else if (minimumJerkProblem->result ().which () ==
	     solver_t::SOLVER_VALUE_WARNINGS)
      {
	const roboptim::Result& result =
	  boost::get<roboptim::ResultWithWarnings>
	  (minimumJerkProblem->result ());
	x = result.x;

	std::stringstream ss;
	ss << result;
	cnoid::callSynchronously
	  (boost::bind
	   (&QMessageBox::setDetailedText,
	    boost::ref (warningBox_), ss.str ().c_str ()));
	cnoid::callSynchronously
	  (boost::bind (&QMessageBox::exec, boost::ref (warningBox_)));

	LOG4CXX_WARN (logger, "solver warnings: " << result);
      }
    else if (minimumJerkProblem->result ().which () == solver_t::SOLVER_VALUE)
      {
	const roboptim::Result& result =
	  boost::get<roboptim::Result> (minimumJerkProblem->result ());
	x = result.x;

	std::stringstream ss;
	ss << result;
	cnoid::callSynchronously
	  (boost::bind
	   (&QMessageBox::setDetailedText,
	    boost::ref (finishedBox_), ss.str ().c_str ()));
	cnoid::callSynchronously
	  (boost::bind (&QMessageBox::exec, boost::ref (finishedBox_)));

	LOG4CXX_WARN (logger, "result: " << result);
      }
    else
      {
	cnoid::MessageView::mainInstance
	  ()->putln ("Roboptim Joint Optimization - failed");
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
	  ()->putln ("Roboptim Joint Optimization - end");
	if (menuItem_)
	  menuItem_->setEnabled (true);
	if (menuItemSolve_)
	  menuItemSolve_->setEnabled (true);
	thread_.reset (); // yeah suicide time.
	return;
      }

    addResultToTree
      (*minimumJerkProblem, bodyMotionItem, "final result", x);
    cnoid::MessageView::mainInstance ()->putln ("Roboptim Joint Optimization - end");
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
  QMessageBox nosolutionBox_;
  QMessageBox warningBox_;
  QMessageBox finishedBox_;

  cnoid::ItemPtr rootItem_;
  cnoid::FolderItem* folderIteration_;

  JointDialog* dialog_;
};

CNOID_IMPLEMENT_PLUGIN_ENTRY (JointPlugin)

void
JointDialog::onAccepted ()
{
  plugin_.solve ();
}
