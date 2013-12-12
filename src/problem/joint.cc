#include <vector>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>

#include <roboptim/core/numeric-linear-function.hh>
#include <roboptim/core/linear-function.hh>
#include <roboptim/core/optimization-logger.hh>
#include <roboptim/core/problem.hh>
#include <roboptim/core/solver.hh>
#include <roboptim/core/solver-factory.hh>
#include <roboptim/core/filter/bind.hh>
#include <roboptim/core/filter/chain.hh>
#include <roboptim/core/filter/selection-by-id.hh>
#include <roboptim/core/filter/selection.hh>
#include <roboptim/core/filter/split.hh>
#include <roboptim/trajectory/state-function.hh>
#include <roboptim/trajectory/vector-interpolation.hh>

#include <roboptim/retargeting/function/choreonoid-body-trajectory.hh>
#include <roboptim/retargeting/function/forward-geometry/choreonoid.hh>
#include <roboptim/retargeting/function/minimum-jerk-trajectory.hh>
#include <roboptim/retargeting/function/torque/choreonoid.hh>
#include <roboptim/retargeting/function/torque/metapod.hh>
#include <roboptim/retargeting/function/zmp/choreonoid.hh>
#include <roboptim/retargeting/function/zmp/metapod.hh>
#include <roboptim/retargeting/function/joint-to-marker/choreonoid.hh>
#include <roboptim/retargeting/function/body-laplacian-deformation-energy/choreonoid.hh>

#include "roboptim/retargeting/problem/joint.hh"

#include <cnoid/Body>
#include "../model/hrp4g2.hh"


static boost::array<double, 6 + 44> standardPose = {{
    0, 0, 0.6, 0, 0, 0, //FIXME: double check robot height

    0, 0, -25, 50, -25, 0, 0,
    0, 0, -25, 50, -25, 0, 0,
    0, 0, 0,
    0, 0, 0,
    -1.0, 1.0, 0, 0, 0, -1.0, 1.0, -1.0,
    5, -10, 0, -15, 0,  10,  1.0, 0,
    5,  10, 0, -15, 0, -10, -1.0, 0
  }};

namespace roboptim
{
  namespace retargeting
  {
    template <typename T, typename U>
    class JointOptimizationLogger : public OptimizationLogger<T>
    {
    public:
      typedef T solver_t;
      typedef typename solver_t::problem_t problem_t;
      typedef typename solver_t::problem_t::value_type value_type;
      typedef typename solver_t::problem_t::vector_t vector_t;
      typedef typename solver_t::problem_t::function_t::discreteInterval_t
      discreteInterval_t;

      explicit JointOptimizationLogger
      (solver_t& solver, const boost::filesystem::path& path,
       discreteInterval_t discreteInterval,
       boost::shared_ptr<Trajectory<3> > trajectory,
       boost::shared_ptr<GenericDifferentiableFunction<U> > zmp,
       typename solver_t::callback_t additionalCallback)
	: OptimizationLogger<T> (solver, path),
	  discreteInterval_ (discreteInterval),
	  trajectory_ (trajectory),
	  zmp_ (zmp),
	  additionalCallback_ (additionalCallback)
      {
      }

      ~JointOptimizationLogger () throw ()
      {}
    protected:
      virtual
      void perIterationCallbackUnsafe
      (const typename solver_t::problem_t& pb,
       typename solver_t::solverState_t& solverState)
      {
	OptimizationLogger<T>::perIterationCallbackUnsafe (pb, solverState);

	boost::filesystem::path path = this->path ();
	const boost::filesystem::path iterationPath =
	  path / (boost::format ("iteration-%d")
		  % this->callbackCallId ()).str ();

	// Log ZMP
	if (zmp_)
	  {
	    boost::filesystem::ofstream streamZmp (iterationPath / "zmp.csv");
	    streamZmp << "ZMP X, ZMP Y\n";

	    const value_type tmin = boost::get<0> (discreteInterval_);
	    const value_type tmax = boost::get<1> (discreteInterval_);
	    const value_type dt = boost::get<2> (discreteInterval_);

	    for (value_type t = tmin; t + dt < tmax; t += dt)
	      {
		trajectory_->setParameters (solverState.x ());
		vector_t state = trajectory_->state (t, 2);
		vector_t zmp = (*zmp_) (state);
		streamZmp << zmp[0] << ", " << zmp[1] << "\n";
	      }
	    streamZmp << "\n";
	  }

	// Call additional callback.
	if (additionalCallback_)
	  additionalCallback_ (pb, solverState);
      }

    private:
      discreteInterval_t discreteInterval_;
      boost::shared_ptr<Trajectory<3> > trajectory_;
      boost::shared_ptr<GenericDifferentiableFunction<U> > zmp_;
      typename solver_t::callback_t additionalCallback_;
    };

    namespace problem
    {
      log4cxx::LoggerPtr Joint::logger
      (log4cxx::Logger::getLogger("roboptim.retargeting.Joint"));

      Joint::Joint
      (cnoid::BodyPtr robot,
       cnoid::BodyMotionPtr initialMotion,
       size_type nNodes,
       bool enableFreezeFrame,
       bool enableVelocity,
       bool enableFeetPositions,
       bool enableCollision,
       bool enableTorque,
       bool enableZmp,
       const std::string& solverName,
       const std::vector<bool>& enabledDofs,
       solver_t::callback_t additionalCallback)
	: robot_ (robot),
	  initialMotion_ (initialMotion),
	  nFrames_ (initialMotion->getNumFrames ()),
	  dt_ (1. / initialMotion->getFrameRate ()),
	  nNodes_ (nNodes),
	  tmin_ (0.),
	  tmax_ (dt_ * static_cast<value_type> (nFrames_)),
	  nDofs_ (std::count (enabledDofs.begin (), enabledDofs.end (), true)),
	  interval_ (tmin_, tmax_, 0.01),
	  cost_ (),
	  positions_ (2),
	  torque_ (),
	  zmp_ (),
	  trajectoryConstraints_ (),
	  problem_ (),
	  result_ (),
	  solverName_ (solverName),
	  additionalCallback_ (additionalCallback),
	  enabledDofs_ (enabledDofs)
      {}

      JointShPtr_t
      Joint::buildVectorInterpolationBasedOptimizationProblem
      (cnoid::BodyPtr robot,
       cnoid::BodyMotionPtr initialMotion,
       cnoid::BodyIMeshPtr mesh,
       bool enableFreezeFrame,
       bool enableVelocity,
       bool enableFeetPositions,
       bool enableCollision,
       bool enableTorque,
       bool enableZmp,
       const std::string& solverName,
       const std::vector<bool>& enabledDofs,
       solver_t::callback_t additionalCallback)
      {
	JointShPtr_t minimumJerk
	  (new Joint
	   (robot,
	    initialMotion,
	    0,
	    enableFreezeFrame,
	    enableVelocity,
	    enableFeetPositions,
	    enableCollision,
	    enableTorque,
	    enableZmp,
	    solverName,
	    enabledDofs,
	    additionalCallback));

	// Compute bound Dofs.
	typedef roboptim::Function::value_type value_type;
	std::vector<boost::optional<value_type> > boundDofs
	  (standardPose.size ());
	for (std::size_t jointId = 0; jointId < standardPose.size (); ++jointId)
	  if (!enabledDofs[jointId])
	    boundDofs[jointId] = standardPose[jointId];

	std::size_t nFrames = initialMotion->getNumFrames ();
	std::vector<bool> enabledDofsAllFrames
	  (nFrames * standardPose.size (), true);
	std::vector<boost::optional<value_type> > boundDofsAllFrames
	  (nFrames * standardPose.size ());
	for (std::size_t frame = 0; frame < nFrames; ++frame)
	  for (std::size_t jointId = 0;
	       jointId < standardPose.size (); ++jointId)
	    {
	      enabledDofsAllFrames[frame * standardPose.size () + jointId] =
		enabledDofs[jointId];
	      if (!enabledDofs[jointId])
		boundDofsAllFrames[frame * standardPose.size () + jointId]
		  = standardPose[jointId];
	    }

        // Compute the initial trajectory (whole body)
	std::size_t nEnabledDofs =
	  std::count (enabledDofs.begin (), enabledDofs.end (), true);
	if (nEnabledDofs == 0)
	  throw std::runtime_error ("all DOFs are disabled");

	// Build the starting point.
	boost::shared_ptr<ChoreonoidBodyTrajectory> initialTrajectory =
	  boost::make_shared<ChoreonoidBodyTrajectory> (initialMotion, true);
	boost::shared_ptr<DifferentiableFunction> initialTrajectoryFct =
	  initialTrajectory;

	// Build the full trajectory.
	vector_t xComplete = initialTrajectory->parameters ();

	// Prune the starting point from useless DOFs
	initialTrajectoryFct =
	  selectionById (initialTrajectoryFct, enabledDofs);

	vector_t x (nEnabledDofs * initialMotion->getNumFrames ());
	vector_t t (1);
	for (int frameId = 0;
	     frameId < initialMotion->getNumFrames (); ++frameId)
	  {
	    t[0] = frameId * (1. / initialMotion->frameRate ());
	    x.segment (frameId * nEnabledDofs, nEnabledDofs) =
	      (*initialTrajectoryFct) (t);
	  }

	// Build the cost function as the difference between the
	// reference trajectory and the current trajectory.
	boost::shared_ptr<JointToMarkerPositionChoreonoid<
	  EigenMatrixDense> >
	  jointToMarkerOrigin =
	  boost::make_shared<JointToMarkerPositionChoreonoid<
	    EigenMatrixDense> > (mesh, 0);
	boost::shared_ptr<DifferentiableFunction>
	  jointToMarker = jointToMarkerOrigin;
	jointToMarker = bind (jointToMarker, boundDofs);

	minimumJerk->cost_ =
	  boost::make_shared<BodyLaplacianDeformationEnergyChoreonoid<
	    EigenMatrixDense> >
	  (mesh, nEnabledDofs, initialMotion->getNumFrames (), x,
	   jointToMarker, jointToMarkerOrigin);
	minimumJerk->cost_ = bind (minimumJerk->cost_, boundDofsAllFrames);

	// Clone the vector interpolation object so that it can be used by
	// constraints
	minimumJerk->trajectoryConstraints_ =
	  boost::make_shared<ChoreonoidBodyTrajectory> (initialMotion, true);

	// Create problem.
	minimumJerk->problem_ =
	  boost::make_shared<problem_t> (*minimumJerk->cost_);
	minimumJerk->problem_->startingPoint () = x;

	minimumJerk->addConstraints
	  (enableFreezeFrame,
	   enableVelocity,
	   enableFeetPositions,
	   enableCollision,
	   enableTorque,
	   enableZmp);

	return minimumJerk;
      }

      JointShPtr_t
      Joint::buildSplineBasedOptimizationProblem
      (cnoid::BodyPtr robot,
       cnoid::BodyMotionPtr initialMotion,
       size_type nNodes,
       bool enableFreezeFrame,
       bool enableVelocity,
       bool enableFeetPositions,
       bool enableCollision,
       bool enableTorque,
       bool enableZmp,
       const std::string& solverName,
       const std::vector<bool>& enabledDofs,
       solver_t::callback_t additionalCallback)
      {
	JointShPtr_t minimumJerk
	  (new Joint
	   (robot,
	    initialMotion,
	    nNodes,
	    enableFreezeFrame,
	    enableVelocity,
	    enableFeetPositions,
	    enableCollision,
	    enableTorque,
	    enableZmp,
	    solverName,
	    enabledDofs,
	    additionalCallback));

	// minimumJerk->addConstraints
	//   (enableFreezeFrame,
	//    enableVelocity,
	//    enableFeetPositions,
	//    enableCollision,
	//    enableTorque,
	//    enableZmp);

	return minimumJerk;
      }

      void
      Joint::addConstraints
      (bool enableFreezeFrame,
       bool enableVelocity,
       bool enableFeetPositions,
       bool enableCollision,
       bool enableTorque,
       bool enableZmp)
      {
	// Should we use Metapod or Choreonoid?
	bool useMetapodTorque = true;
	bool useMetapodZmp = false;

	std::size_t nEnabledDofs =
	  std::count (enabledDofs_.begin (), enabledDofs_.end (), true);
	if (nEnabledDofs == 0)
	  throw std::runtime_error ("all DOFs are disabled");

	// Compute bound Dofs.
	typedef roboptim::Function::value_type value_type;
	std::vector<boost::optional<value_type> > boundDofs
	  (standardPose.size ());
	for (std::size_t jointId = 0; jointId < standardPose.size (); ++jointId)
	  if (!enabledDofs_[jointId])
	    boundDofs[jointId] = standardPose[jointId];

	std::vector<bool> enabledDofsAllFrames
	  (nFrames_ * standardPose.size (), true);
	std::vector<boost::optional<value_type> > boundDofsAllFrames
	  (nFrames_ * standardPose.size ());
	for (std::size_t frame = 0; frame < nFrames_; ++frame)
	  for (std::size_t jointId = 0;
	       jointId < standardPose.size (); ++jointId)
	    {
	      enabledDofsAllFrames[frame * standardPose.size () + jointId] =
		enabledDofs_[jointId];
	      if (!enabledDofs_[jointId])
		boundDofsAllFrames[frame * standardPose.size () + jointId]
		  = standardPose[jointId];
	    }

	// Compute enabled and bound Dofs for state functions using
	// first derivative.
	std::vector<bool> enabledDofsStateFunction2 (standardPose.size () * 2);
	for (std::size_t jointId = 0; jointId < standardPose.size (); ++jointId)
	  {
	    enabledDofsStateFunction2[0 * standardPose.size () + jointId]
	      = enabledDofs_[jointId];
	    enabledDofsStateFunction2[1 * standardPose.size () + jointId]
	      = enabledDofs_[jointId];
	  }

	std::vector<boost::optional<value_type> > boundDofsStateFunction2
	  (standardPose.size () * 2);
	for (std::size_t jointId = 0; jointId < standardPose.size (); ++jointId)
	  if (!enabledDofs_[jointId])
	    {
	      boundDofsStateFunction2[0 * standardPose.size () + jointId]
		= standardPose[jointId];
	      boundDofsStateFunction2[1 * standardPose.size () + jointId]
		= 0.;
	    }

	// Compute enabled and bound Dofs for state functions using first and
	// second order derivatives.
	std::vector<bool> enabledDofsStateFunction3 (standardPose.size () * 3);
	for (std::size_t jointId = 0; jointId < standardPose.size (); ++jointId)
	  {
	    enabledDofsStateFunction3[0 * standardPose.size () + jointId]
	      = enabledDofs_[jointId];
	    enabledDofsStateFunction3[1 * standardPose.size () + jointId]
	      = enabledDofs_[jointId];
	    enabledDofsStateFunction3[2 * standardPose.size () + jointId]
	      = enabledDofs_[jointId];
	  }

	std::vector<boost::optional<value_type> > boundDofsStateFunction3
	  (standardPose.size () * 3);
	for (std::size_t jointId = 0; jointId < standardPose.size (); ++jointId)
	  if (!enabledDofs_[jointId])
	    {
	      boundDofsStateFunction3[0 * standardPose.size () + jointId]
		= standardPose[jointId];
	      boundDofsStateFunction3[1 * standardPose.size () + jointId]
		= 0.;
	      boundDofsStateFunction3[2 * standardPose.size () + jointId]
		= 0.;
	    }


	// Bound joint positions.
	{
	  for (std::size_t frameId = 0; frameId < nFrames_; ++frameId)
	    {
	      std::size_t id = 0;
	      for (std::size_t jointId = 0;
		   jointId < 6 + robot_->numJoints (); ++jointId)
		{
		  if (!enabledDofs_[jointId])
		    continue;
		  if (jointId < 6)
		    {
		      id++;
		      continue;
		    }

		  problem_->argumentBounds ()
		    [frameId * nEnabledDofs + id] =
		    MinimumJerkTrajectory<EigenMatrixDense>::makeInterval
		    (robot_->joint (jointId - 6)->q_lower (),
		     robot_->joint (jointId - 6)->q_upper ());
		  ++id;
		}
	    }
	}

	// Freeze first frame
	if (enableFreezeFrame)
	  {
	    matrix_t A (nDofs_, nFrames_ * nDofs_);
	    A.setZero ();
	    A.block (0, 0, nDofs_, nDofs_).setIdentity ();
	    vector_t b (nDofs_);

	    std::size_t jointIdFiltered = 0;
	    for (std::size_t jointId = 0; jointId < standardPose.size (); ++jointId)
	      {
		if (!enabledDofs_[jointId])
		  continue;
		b[jointIdFiltered++] = (jointId < 6)
		  ? -standardPose[jointId] : -standardPose[jointId] * M_PI / 180.;
	      }
	    assert (jointIdFiltered == b.size ());

	    freeze_ =
	      boost::make_shared<
		GenericNumericLinearFunction<EigenMatrixDense> >
	      (A, b);

	    std::vector<interval_t> freezeBounds
	      (freeze_->outputSize (), Function::makeInterval (0., 0.));
	    std::vector<value_type> freezeScales (freeze_->outputSize (), 1.);

	    problem_->addConstraint (freeze_, freezeBounds, freezeScales);
	  }

	if (enableFeetPositions)
	  {
	    unsigned nConstraints = nFrames_ - 2;

	    vector_t qInitial (6 + robot_->numJoints ());
	    for (std::size_t dofId = 0; dofId < qInitial.size (); ++dofId)
	      qInitial[dofId] = standardPose[dofId];

	    typedef ForwardGeometryChoreonoid<EigenMatrixDense>
	      forwardGeometry_t;

	    // Left foot.
	    positions_[0] =
	      boost::make_shared<forwardGeometry_t>
	      (robot_, std::string ("L_TOE_P"));

	    forwardGeometry_t::vector_t leftFootPosition =
	      (*positions_[0]) (qInitial);
	    std::vector<interval_t> leftFootBounds (leftFootPosition.size ());
	    for (std::size_t i = 0; i < leftFootPosition.size (); ++i)
	      leftFootBounds[i] = forwardGeometry_t::makeInterval
		(leftFootPosition[i], leftFootPosition[i]);
	    std::vector<value_type> leftFootScales (leftFootPosition.size (), 1.);

	    // Bind to filter dofs.
	    positions_[0] = bind (positions_[0], boundDofs);

	    roboptim::StateFunction<Trajectory<3> >::addToProblem
	      (*trajectoryConstraints_, positions_[0], 0,
	       *problem_, leftFootBounds, leftFootScales, nConstraints);

	    // Right foot.
	    positions_[1] =
	      boost::make_shared<forwardGeometry_t>
	      (robot_, std::string ("R_TOE_P"));

	    forwardGeometry_t::vector_t rightFootPosition =
	      (*positions_[1]) (qInitial);
	    std::vector<interval_t> rightFootBounds (rightFootPosition.size ());
	    for (std::size_t i = 0; i < rightFootPosition.size (); ++i)
	      rightFootBounds[i] = forwardGeometry_t::makeInterval
		(rightFootPosition[i], rightFootPosition[i]);
	    std::vector<value_type> rightFootScales
	      (rightFootPosition.size (), 1.);

	    // Bind to filter dof.
	    positions_[1] = bind (positions_[1], boundDofs);

	    roboptim::StateFunction<Trajectory<3> >::addToProblem
	      (*trajectoryConstraints_, positions_[1], 0,
	       *problem_, rightFootBounds, rightFootScales, nConstraints);
	  }

	boost::shared_ptr<GenericLinearFunction<EigenMatrixDense> >
	  velocityOneFrame;
	if (enableVelocity)
	  {
	    unsigned nConstraints = nFrames_ - 2;

	    matrix_t A (6 + robot_->numJoints (),
			2 * (6 + robot_->numJoints ()));
	    A.setZero ();
	    A.block
	      (0, 6 + robot_->numJoints (),
	       6 + robot_->numJoints (),
	       6 + robot_->numJoints ()).setIdentity ();
	    vector_t b (6 + robot_->numJoints ());
	    b.setZero ();
	    velocityOneFrame =
	      boost::make_shared<
		GenericNumericLinearFunction<EigenMatrixDense> >
	      (A, b);

	    // Bind and select to filter dofs.
	    velocityOneFrame = selectionById (velocityOneFrame,
					      enabledDofs_);
	    velocityOneFrame = bind (velocityOneFrame, boundDofsStateFunction2);

	    std::vector<interval_t> velocityBounds
	      (velocityOneFrame->outputSize ());
	    std::size_t id = 0;
	    for (std::size_t jointId = 0;
		 jointId < robot_->numJoints (); ++jointId)
	      {
		if (jointId < 6)
		  {
		    ++id;
		    continue;
		  }
		if (enabledDofs_[jointId])
		  velocityBounds[id++] = Function::makeInterval
		    (robot_->joint (jointId - 6)->dq_lower (),
		     robot_->joint (jointId - 6)->dq_upper ());
	      }
	    std::vector<value_type> velocityScales
	      (velocityOneFrame->outputSize (), 1.);

	    roboptim::StateFunction<Trajectory<3> >::addToProblem
	      (*trajectoryConstraints_, velocityOneFrame, 1,
	       *problem_, velocityBounds, velocityScales, nConstraints);
	  }
	if (enableTorque)
	  {
	    unsigned nConstraints = nFrames_ - 2;

	    if (useMetapodTorque)
	      torque_ =
		boost::make_shared<TorqueMetapod<
		  EigenMatrixDense, metapod::hrp4g2<double> > > ();
	    else
	      torque_ =
		boost::make_shared<TorqueChoreonoid<EigenMatrixDense> > (robot_);

	    // Bind and select to filter dofs.
	    torque_ = bind (torque_, boundDofsStateFunction3);
	    torque_ = selectionById (torque_, enabledDofsStateFunction3);


	    std::vector<interval_t> torqueBounds (6 + robot_->numJoints ());
	    // FIXME: should we constraint the free floating?
	    // static const double g = 9.81;
	    torqueBounds[0] = std::make_pair (-Function::infinity(), Function::infinity()); // FREE FLOATING X
	    torqueBounds[1] = std::make_pair (-Function::infinity(), Function::infinity()); // FREE FLOATING Y
	    torqueBounds[2] = std::make_pair (-Function::infinity(), Function::infinity()); // FREE FLOATING Z
	    torqueBounds[3] = std::make_pair (-Function::infinity(), Function::infinity()); // FREE FLOATING ROLL
	    torqueBounds[4] = std::make_pair (-Function::infinity(), Function::infinity()); // FREE FLOATING PITCH
	    torqueBounds[5] = std::make_pair (-Function::infinity(), Function::infinity()); // FREE FLOATING YAW

	    torqueBounds[6 + 0] = std::make_pair (-63.55, 63.55); // R_HIP_Y
	    torqueBounds[6 + 1] = std::make_pair (-186.21, 186.21); // R_HIP_R
	    torqueBounds[6 + 2] = std::make_pair (-95.18, 95.18); // R_HIP_P
	    torqueBounds[6 + 3] = std::make_pair (-145.98, 145.98); // R_KNEE_P
	    torqueBounds[6 + 4] = std::make_pair (-111.42, 111.42); // R_ANKLE_P
	    torqueBounds[6 + 5] = std::make_pair (-75.11, 75.11); // R_ANKLE_R
	    torqueBounds[6 + 6] = std::make_pair (-52.78, 52.78); // R_TOE_P
	    torqueBounds[6 + 7] = std::make_pair (-186.21, 186.21); // L_HIP_R
	    torqueBounds[6 + 8] = std::make_pair (-151.4, 151.4); // L_HIP_Y
	    torqueBounds[6 + 9] = std::make_pair (-95.18, 95.18); // L_HIP_P
	    torqueBounds[6 + 10] = std::make_pair (-145.98, 145.98); // L_KNEE_P
	    torqueBounds[6 + 11] = std::make_pair (-111.42, 111.42); // L_ANKLE_P
	    torqueBounds[6 + 12] = std::make_pair (-75.11, 75.11); // L_ANKLE_R
	    torqueBounds[6 + 13] = std::make_pair (-52.78, 52.78); // L_TOE_P
	    torqueBounds[6 + 14] = std::make_pair (-97.53, 97.53); // CHEST_P
	    torqueBounds[6 + 15] = std::make_pair (-96.93, 96.93); // CHEST_R
	    torqueBounds[6 + 16] = std::make_pair (-90.97, 90.97); // CHEST_Y
	    torqueBounds[6 + 17] = std::make_pair (-17.59, 17.59); // NECK_Y
	    torqueBounds[6 + 18] = std::make_pair (-17.59, 17.59); // NECK_R
	    torqueBounds[6 + 19] = std::make_pair (-17.59, 17.59); // NECK_P
	    torqueBounds[6 + 20] = std::make_pair (-5.26, 5.26); // EYEBROW_P
	    torqueBounds[6 + 21] = std::make_pair (-0.71, 0.71); // EYELID_P
	    torqueBounds[6 + 22] = std::make_pair (-0.84, 0.84); // EYE_P
	    torqueBounds[6 + 23] = std::make_pair (-0.42, 0.42); // EYE_Y
	    torqueBounds[6 + 24] = std::make_pair (-4.72, 4.72); // MOUTH_P
	    torqueBounds[6 + 25] = std::make_pair (-0.22, 0.22); // LOWERLIP_P
	    torqueBounds[6 + 26] = std::make_pair (-0.29, 0.29); // UPPERLIP_P
	    torqueBounds[6 + 27] = std::make_pair (-5.9, 5.9); // CHEEK_P
	    torqueBounds[6 + 28] = std::make_pair (-181.74, 181.74); // R_SHOULDER_P
	    torqueBounds[6 + 29] = std::make_pair (-62.83, 62.83); // R_SHOULDER_R
	    torqueBounds[6 + 30] = std::make_pair (-20.47, 20.47); // R_SHOULDER_Y
	    torqueBounds[6 + 31] = std::make_pair (-54.46, 54.46); // R_ELBOW_P
	    torqueBounds[6 + 32] = std::make_pair (-6.33, 6.33); // R_WRIST_Y
	    torqueBounds[6 + 33] = std::make_pair (-6.33, 6.33); // R_WRIST_R
	    torqueBounds[6 + 34] = std::make_pair (-0.77, 0.77); // R_HAND_J0
	    torqueBounds[6 + 35] = std::make_pair (-1.16, 1.16); // R_HAND_J1
	    torqueBounds[6 + 36] = std::make_pair (-181.74, 181.74); // L_SHOULDER_P
	    torqueBounds[6 + 37] = std::make_pair (-62.83, 62.83); // L_SHOULDER_R
	    torqueBounds[6 + 38] = std::make_pair (-20.47, 20.47); // L_SHOULDER_Y
	    torqueBounds[6 + 39] = std::make_pair (-54.46, 54.46); // L_ELBOW_P
	    torqueBounds[6 + 40] = std::make_pair (-6.33, 6.33); // L_WRIST_Y
	    torqueBounds[6 + 41] = std::make_pair (-6.33, 6.33); // L_WRIST_R
	    torqueBounds[6 + 42] = std::make_pair (-0.77, 0.77); // L_HAND_J0
	    torqueBounds[6 + 43] = std::make_pair (-1.16, 1.16); // L_HAND_J1

	    std::vector<value_type> torqueScales (torque_->outputSize (), 1.);
	    std::vector<interval_t> torqueBoundsFiltered (torque_->outputSize ());

	    std::size_t id = 0;
	    for (std::size_t jointId = 0;
		 jointId < torque_->outputSize (); ++jointId)
	      if (enabledDofs_[jointId])
		torqueBoundsFiltered[id++] = torqueBounds[jointId];

	    roboptim::StateFunction<Trajectory<3> >::addToProblem
	      (*trajectoryConstraints_, torque_, 2,
	       *problem_, torqueBoundsFiltered, torqueScales, nConstraints);
	  }
	if (enableZmp)
	  {
	    unsigned nConstraints = nFrames_ - 2;
	    value_type soleX = 0.03;
	    value_type soleY = 0.;
	    value_type soleLength = 0.2; //FIXME:
	    value_type soleWidth = 0.1; //FIXME:

	    std::vector<interval_t> zmpBounds (2);
	    zmpBounds[0] = interval_t (soleX - .5 * soleLength,
				       soleX + .5 * soleLength);
	    zmpBounds[1] = interval_t (soleY - .5 * soleWidth,
				       soleY + .5 * soleWidth);

	    std::vector<value_type> zmpScales (2, 1.);

	    if (useMetapodZmp)
	      zmp_ =
		boost::make_shared<
		  ZMPMetapod<EigenMatrixDense, metapod::hrp4g2<double> > > ();
	    else
	      zmp_ =
		boost::make_shared<ZMPChoreonoid<EigenMatrixDense> > (robot_);

	    // Bind and select to filter dofs.
	    zmp_ = bind (zmp_, boundDofsStateFunction3);

	    roboptim::StateFunction<Trajectory<3> >::addToProblem
	      (*trajectoryConstraints_, zmp_, 2,
	       *problem_, zmpBounds, zmpScales, nConstraints);
	  }
      }

      Joint::~Joint ()
      {
      }

      void
      Joint::solve ()
      {
	roboptim::SolverFactory<solver_t> factory (solverName_, *problem_);
	solver_t& solver = factory ();

	boost::shared_ptr<Trajectory<3> >
	  trajectory =
	  vectorInterpolation
	  (*problem_->startingPoint (),
	   static_cast<size_type> (nDofs_), dt_);
	JointOptimizationLogger<solver_t, EigenMatrixDense>
	  optimizationLogger
	  (solver, "/tmp/minimum-jerk-problem",
	   interval_, trajectory, zmp_, additionalCallback_);

	// Set solver parameters.
	solver.parameters ()["max-iterations"].value = 100;
	solver.parameters ()["ipopt.output_file"].value =
	  "/tmp/ipopt.log";
	solver.parameters ()["ipopt.expect_infeasible_problem"].value = "yes";
	solver.parameters ()["ipopt.nlp_scaling_method"].value = "none";
	solver.parameters ()["nag.verify-level"].value = 0;

	std::cerr << solver << std::endl;

	LOG4CXX_INFO (logger, "Solver:\n" << solver);
	LOG4CXX_DEBUG(logger, "start solving...");
	solver.solve ();
	LOG4CXX_DEBUG(logger, "problem solved.");
	result_ = solver.minimum ();
      }

    } // end of namespace problem.
  } // end of namespace retargeting.
} // end of namespace roboptim.
