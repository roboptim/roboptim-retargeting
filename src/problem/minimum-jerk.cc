#include <vector>
#include <boost/make_shared.hpp>

#include <roboptim/core/numeric-linear-function.hh>
#include <roboptim/core/linear-function.hh>
#include <roboptim/core/optimization-logger.hh>
#include <roboptim/core/problem.hh>
#include <roboptim/core/solver.hh>
#include <roboptim/core/solver-factory.hh>
#include <roboptim/core/filter/chain.hh>
#include <roboptim/core/filter/selection.hh>
#include <roboptim/core/filter/split.hh>
#include <roboptim/trajectory/state-function.hh>
#include <roboptim/trajectory/vector-interpolation.hh>

#include <roboptim/retargeting/function/forward-geometry/choreonoid.hh>
#include <roboptim/retargeting/function/torque/choreonoid.hh>
#include <roboptim/retargeting/function/torque/metapod.hh>
#include <roboptim/retargeting/function/zmp/choreonoid.hh>
#include <roboptim/retargeting/function/zmp/metapod.hh>


#include "roboptim/retargeting/problem/minimum-jerk.hh"

#include <cnoid/Body>
#include "../model/hrp4g2.hh"


boost::array<double, 6 + 44> standardPose = {{
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
    class MinimumJerkOptimizationLogger : public OptimizationLogger<T>
    {
    public:
      typedef T solver_t;
      typedef typename solver_t::problem_t problem_t;
      typedef typename solver_t::problem_t::value_type value_type;
      typedef typename solver_t::problem_t::vector_t vector_t;
      typedef typename solver_t::problem_t::function_t::discreteInterval_t
      discreteInterval_t;

      explicit MinimumJerkOptimizationLogger
      (solver_t& solver, const boost::filesystem::path& path,
       discreteInterval_t discreteInterval,
       boost::shared_ptr<Trajectory<3> > trajectory,
       boost::shared_ptr<ZMP<U> > zmp,
       typename solver_t::callback_t additionalCallback)
	: OptimizationLogger<T> (solver, path),
	  discreteInterval_ (discreteInterval),
	  trajectory_ (trajectory),
	  zmp_ (zmp),
	  additionalCallback_ (additionalCallback)
      {
      }

      ~MinimumJerkOptimizationLogger () throw ()
      {}
    protected:
      virtual
      void perIterationCallbackUnsafe (const typename solver_t::vector_t& x,
				       const typename solver_t::problem_t& pb)
      {
	OptimizationLogger<T>::perIterationCallbackUnsafe (x, pb);

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
		trajectory_->setParameters (x);
		vector_t state = trajectory_->state (t, 2);
		vector_t zmp = (*zmp_) (state);
		streamZmp << zmp[0] << ", " << zmp[1] << "\n";
	      }
	    streamZmp << "\n";
	  }

	// Call additional callback.
	if (additionalCallback_)
	  additionalCallback_ (x, pb);
      }

    private:
      discreteInterval_t discreteInterval_;
      boost::shared_ptr<Trajectory<3> > trajectory_;
      boost::shared_ptr<ZMP<U> > zmp_;
      typename solver_t::callback_t additionalCallback_;
    };

    namespace problem
    {
      log4cxx::LoggerPtr MinimumJerk::logger
      (log4cxx::Logger::getLogger("roboptim.retargeting.MinimumJerk"));

      MinimumJerk::MinimumJerk
      (cnoid::BodyPtr robot,
       bool enableFreezeFrame,
       bool enableVelocity,
       bool enableFeetPositions,
       bool enableCollision,
       bool enableTorque,
       bool enableZmp,
       const std::string& solverName,
       solver_t::callback_t additionalCallback)
	: nFrames_ (10.),
	  dt_ (0.005),
	  tmin_ (0.),
	  tmax_ (dt_ * static_cast<value_type> (nFrames_)),
	  dofId_ (6 + 28),
	  init_ (standardPose[dofId_] * M_PI / 180.),
	  goal_ (robot->joint (dofId_ - 6)->q_upper () / 5. - 0.01),
	  nDofs_ (standardPose.size ()),
	  interval_ (tmin_, tmax_, 0.01),
	  cost_ (),
	  positions_ (2),
	  torque_ (),
	  zmp_ (),
	  vectorInterpolationConstraints_ (),
	  problem_ (),
	  result_ (),
	  solverName_ (solverName),
	  additionalCallback_ (additionalCallback)
      {
        // Compute the initial trajectory (whole body)
	vector_t initialTrajectory (nFrames_ * standardPose.size ());
	initialTrajectory.setZero ();

	vector_t x (4);
	x[0] = init_;
	x[1] = goal_;
	x[2] = x[3] = 0.;

	boost::shared_ptr<MinimumJerkTrajectory<EigenMatrixDense> >
	  minimumJerkTrajectory =
	  boost::make_shared<MinimumJerkTrajectory<EigenMatrixDense> >
	  ();
	minimumJerkTrajectory->setParameters (x);

	// Build the starting point
	value_type dtMinimumJerk = (1. - 0.) / nFrames_;
	for (std::size_t frameId = 0; frameId < nFrames_; ++frameId)
	  {
	    for (std::size_t dof = 0; dof < nDofs_; ++dof)
	      initialTrajectory[frameId * nDofs_ + dof] =
		(dof < 6)
		? standardPose[dof] : standardPose[dof] * M_PI / 180.;

	    initialTrajectory[frameId * nDofs_ + dofId_] =
	      (*minimumJerkTrajectory)
	      (static_cast<value_type> (frameId) * dtMinimumJerk)[0];
	  }

	// Build a trajectory over the starting point for display
	boost::shared_ptr<VectorInterpolation >
	  initialTrajectoryFct =
	  vectorInterpolation
	  (initialTrajectory, static_cast<size_type> (nDofs_), dt_);

	// Build the cost function as the difference between the
	// reference trajectory and the current trajectory.
	cost_ = boost::make_shared<CostReferenceTrajectory<EigenMatrixDense> >
	  (initialTrajectoryFct, dofId_, dt_);

	// Clone the vector interpolation object so that it can be used by
	// constraints
	vectorInterpolationConstraints_ =
	  vectorInterpolation
	  (initialTrajectory, static_cast<size_type> (nDofs_), dt_);


	// Create problem.
	problem_ = boost::make_shared<problem_t> (*cost_);
	problem_->startingPoint () = initialTrajectory;

	// Add constraints.
	// Should we use Metapod or Choreonoid?
	bool useMetapod = false;

	// Bound joint positions and free first and last frames.
	{
	  for (std::size_t frameId = 0; frameId < nFrames_; ++frameId)
	    for (std::size_t jointId = 0;
		 jointId < robot->numJoints (); ++jointId)
	      problem_->argumentBounds ()
		[frameId * (6 + robot->numJoints ()) + 6 + jointId] =
		MinimumJerkTrajectory<EigenMatrixDense>::makeInterval
		(robot->joint (jointId)->q_lower (),
		 robot->joint (jointId)->q_upper ());
	}

	// Freeze first frame
	if (enableFreezeFrame)
	  {
	    matrix_t A (6 + robot->numJoints (),
			nFrames_ * (6 + robot->numJoints ()));
	    A.setZero ();
	    A.block (0, 0,
		     6 + robot->numJoints (),
		     6 + robot->numJoints ()).setIdentity ();
	    vector_t b (6 + robot->numJoints ());
	    for (std::size_t jointId = 0; jointId < b.size (); ++jointId)
	      b[jointId] = (jointId < 6)
		? -standardPose[jointId] : -standardPose[jointId] * M_PI / 180.;

	    freeze_ =
	      boost::make_shared<
		GenericNumericLinearFunction<EigenMatrixDense> >
	      (A, b);

	    std::vector<interval_t> freezeBounds (6 + robot->numJoints ());
	    for (std::size_t jointId = 0;
		 jointId < 6 + robot->numJoints (); ++jointId)
	      freezeBounds[jointId] = Function::makeInterval (0., 0.);

	    std::vector<value_type> freezeScales (6 + robot->numJoints ());
	    for (std::size_t jointId = 0;
		 jointId < 6 + robot->numJoints (); ++jointId)
	      freezeScales[jointId] = 1.;

	    problem_->addConstraint (freeze_, freezeBounds, freezeScales);
	  }

	if (enableFeetPositions)
	  {
	    unsigned nConstraints = nFrames_ - 2;

	    typedef ForwardGeometryChoreonoid<EigenMatrixDense>
	      forwardGeometry_t;

	    // Left foot.
	    positions_[0] =
	      boost::make_shared<forwardGeometry_t>
	      (robot, std::string ("L_TOE_P"));

	    forwardGeometry_t::vector_t leftFootPosition =
	      (*positions_[0]) ((*initialTrajectoryFct) (0.));
	    std::vector<interval_t> leftFootBounds (leftFootPosition.size ());
	    for (std::size_t i = 0; i < leftFootPosition.size (); ++i)
	      leftFootBounds[i] = forwardGeometry_t::makeInterval
		(leftFootPosition[i] - 1e-4, leftFootPosition[i] + 1e-4);

	    std::vector<value_type> leftFootScales (6);
	    for (std::size_t i = 0; i < 6; ++i)
	      leftFootScales[i] = 1.;

	    roboptim::StateFunction<VectorInterpolation>::addToProblem
	      (*vectorInterpolationConstraints_, positions_[0], 0,
	       *problem_, leftFootBounds, leftFootScales, nConstraints);

	    // Right foot.
	    positions_[1] =
	      boost::make_shared<forwardGeometry_t>
	      (robot, std::string ("R_TOE_P"));

	    forwardGeometry_t::vector_t rightFootPosition =
	      (*positions_[1]) ((*initialTrajectoryFct) (0.));
	    std::vector<interval_t> rightFootBounds (rightFootPosition.size ());
	    for (std::size_t i = 0; i < rightFootPosition.size (); ++i)
	      rightFootBounds[i] = forwardGeometry_t::makeInterval
		(rightFootPosition[i] - 1e-4, rightFootPosition[i] + 1e-4);
	    std::vector<value_type> rightFootScales (6);
	    for (std::size_t i = 0; i < 6; ++i)
	      rightFootScales[i] = 1.;

	    roboptim::StateFunction<VectorInterpolation>::addToProblem
	      (*vectorInterpolationConstraints_, positions_[1], 0,
	       *problem_, rightFootBounds, rightFootScales, nConstraints);
	  }

	boost::shared_ptr<GenericLinearFunction<EigenMatrixDense> >
	  velocityOneFrame;
	if (enableVelocity)
	  {
	    unsigned nConstraints = nFrames_ - 2;

	    matrix_t A (robot->numJoints (),
			2 * (6 + robot->numJoints ()));
	    A.setZero ();
	    A.block
	      (0, 6 + robot->numJoints (),
	       robot->numJoints (), robot->numJoints ()).setIdentity ();
	    vector_t b (robot->numJoints ());
	    b.setZero ();
	    velocityOneFrame =
	      boost::make_shared<
		GenericNumericLinearFunction<EigenMatrixDense> >
	      (A, b);

	    std::vector<interval_t> velocityBounds (robot->numJoints ());
	    for (std::size_t jointId = 0;
		 jointId < robot->numJoints (); ++jointId)
	      velocityBounds[jointId] = Function::makeInterval
		(robot->joint (jointId)->dq_lower (),
		 robot->joint (jointId)->dq_upper ());

	    std::vector<value_type> velocityScales (robot->numJoints ());
	    for (std::size_t jointId = 0;
		 jointId < robot->numJoints (); ++jointId)
	      velocityScales[jointId] = 1.;

	    roboptim::StateFunction<VectorInterpolation>::addToProblem
	      (*vectorInterpolationConstraints_, velocityOneFrame, 1,
	       *problem_, velocityBounds, velocityScales, nConstraints);
	  }
	if (enableTorque)
	  {
	    unsigned nConstraints = nFrames_ - 2;
	    std::vector<interval_t> torqueBounds (nDofs_);
	    for (std::size_t i = 0; i < nDofs_; ++i)
	      torqueBounds[i] = interval_t (-1, 1);

	    std::vector<value_type> torqueScales (nDofs_);
	    for (std::size_t i = 0; i < nDofs_; ++i)
	      torqueScales[i] = 1.;

	    if (useMetapod)
	      torque_ =
		boost::make_shared<TorqueMetapod<
		  EigenMatrixDense, metapod::hrp4g2> > ();
	    else
	      torque_ =
		boost::make_shared<TorqueChoreonoid<EigenMatrixDense> > (robot);

	    roboptim::StateFunction<VectorInterpolation>::addToProblem
	      (*vectorInterpolationConstraints_, torque_, 2,
	       *problem_, torqueBounds, torqueScales, nConstraints);
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

	    std::vector<value_type> zmpScales (2);
	    zmpScales[0] = 1.;
	    zmpScales[1] = 1.;

	    if (useMetapod)
	      zmp_ =
		boost::make_shared<
		  ZMPMetapod<EigenMatrixDense, metapod::hrp4g2> > ();
	    else
	      zmp_ =
		boost::make_shared<ZMPChoreonoid<EigenMatrixDense> > (robot);

	    roboptim::StateFunction<VectorInterpolation>::addToProblem
	      (*vectorInterpolationConstraints_, zmp_, 2,
	       *problem_, zmpBounds, zmpScales, nConstraints);
	  }
      }

      MinimumJerk::~MinimumJerk ()
      {
      }

      void
      MinimumJerk::solve ()
      {
	roboptim::SolverFactory<solver_t> factory (solverName_, *problem_);
	solver_t& solver = factory ();

	boost::shared_ptr<VectorInterpolation >
	  trajectory =
	  vectorInterpolation
	  (*problem_->startingPoint (),
	   static_cast<size_type> (nDofs_), dt_);
	MinimumJerkOptimizationLogger<solver_t, EigenMatrixDense>
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
