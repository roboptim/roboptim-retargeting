#ifndef ROBOPTIM_RETARGETING_PROBLEM_JOINT_HH
# define ROBOPTIM_RETARGETING_PROBLEM_JOINT_HH
# include <string>

# include <boost/function.hpp>
# include <boost/shared_ptr.hpp>

# include <roboptim/core/differentiable-function.hh>
# include <roboptim/core/generic-solver.hh>
# include <roboptim/core/linear-function.hh>
# include <roboptim/core/problem.hh>
# include <roboptim/core/solver.hh>

# include <roboptim/retargeting/function/cost-reference-trajectory.hh>
# include <roboptim/retargeting/function/forward-geometry.hh>
# include <roboptim/retargeting/function/minimum-jerk-trajectory.hh>
# include <roboptim/retargeting/function/torque.hh>
# include <roboptim/retargeting/function/zmp.hh>

# include <cnoid/Body>

namespace roboptim
{
  namespace retargeting
  {
    namespace problem
    {
      class Joint;

      typedef boost::shared_ptr<Joint> JointShPtr_t;

      /// \brief Define minimum jerk optimization problem.
      class Joint
      {
      public:
	// Forward typedefs.
	typedef MinimumJerkTrajectory<EigenMatrixDense>::matrix_t matrix_t;
	typedef MinimumJerkTrajectory<EigenMatrixDense>::vector_t vector_t;
	typedef MinimumJerkTrajectory<EigenMatrixDense>::interval_t interval_t;
	typedef MinimumJerkTrajectory<EigenMatrixDense>::value_type value_type;
	typedef MinimumJerkTrajectory<EigenMatrixDense>::size_type size_type;

	typedef boost::shared_ptr<GenericLinearFunction<EigenMatrixDense> >
	LinearFunctionShPtr_t;
	typedef boost::shared_ptr<ForwardGeometry<EigenMatrixDense> >
	ForwardGeometryShPtr_t;
	typedef boost::shared_ptr<Torque<EigenMatrixDense> > TorqueShPtr_t;
	typedef boost::shared_ptr<ZMP<EigenMatrixDense> > ZmpShPtr_t;
	typedef boost::shared_ptr<
	  roboptim::GenericDifferentiableFunction<
	    EigenMatrixDense> >
	DifferentiableFunctionShPtr_t;

	typedef roboptim::Solver<
	  GenericDifferentiableFunction<EigenMatrixDense>,
	  boost::mpl::vector<
	    GenericLinearFunction<EigenMatrixDense>,
	    GenericDifferentiableFunction<EigenMatrixDense> >
	  >
	solver_t;
	typedef solver_t::problem_t problem_t;

	typedef boost::shared_ptr<problem_t>
	ProblemShPtr_t;

	static JointShPtr_t
	buildVectorInterpolationBasedOptimizationProblem
	(cnoid::BodyPtr robot,
	 size_type nFrames,
	 value_type dt,
	 bool enableFreezeFrame,
	 bool enableVelocity,
	 bool enablePosition,
	 bool enableCollision,
	 bool enableTorque,
	 bool enableZmp,
	 const std::string& solverName,
	 const std::vector<bool>& enabledDofs,
	 solver_t::callback_t additionalCallback = solver_t::callback_t ());

	static JointShPtr_t
	buildSplineBasedOptimizationProblem
	(cnoid::BodyPtr robot,
	 size_type nFrames,
	 size_type nNodes,
	 bool enableFreezeFrame,
	 bool enableVelocity,
	 bool enablePosition,
	 bool enableCollision,
	 bool enableTorque,
	 bool enableZmp,
	 const std::string& solverName,
	 const std::vector<bool>& enabledDofs,
	 solver_t::callback_t additionalCallback = solver_t::callback_t ());

	virtual ~Joint ();

	/// \brief Solve the underlying optimization problem.
	void solve ();

	const DifferentiableFunctionShPtr_t& cost () const
	{
	  return cost_;
	}

	DifferentiableFunctionShPtr_t& cost ()
	{
	  return cost_;
	}

	ProblemShPtr_t& problem ()
	{
	  return problem_;
	}

	const ProblemShPtr_t& problem () const
	{
	  return problem_;
	}

	/// \brief Return the optimization result.
	///
	/// You *have* to call solve() first otherwise
	/// the result will contain no information.
	const GenericSolver::result_t& result () const
	{
	  return result_;
	}

	DifferentiableFunctionShPtr_t torqueConstraint ()
	{
	  return torque_;
	}

	std::size_t nFrames () const throw ()
	{
	  return nFrames_;
	}

	std::size_t nDofs () const throw ()
	{
	  return nDofs_;
	}

	value_type dt () const throw ()
	{
	  return dt_;
	}

	MinimumJerkTrajectory<EigenMatrixDense>::discreteInterval_t
	interval () const throw ()
	{
	  return interval_;
	}

	solver_t::callback_t&
	additionalCallback () throw ()
	{
	  return additionalCallback_;
	}

	const solver_t::callback_t&
	additionalCallback () const throw ()
	{
	  return additionalCallback_;
	}

      protected:
	explicit Joint
	(cnoid::BodyPtr robot,
	 size_type nFrames,
	 value_type dt,
	 size_type nNodes,
	 bool enableFreezeFrame,
	 bool enableVelocity,
	 bool enableFeetPositions,
	 bool enableCollision,
	 bool enableTorque,
	 bool enableZmp,
	 const std::string& solverName,
	 const std::vector<bool>& enabledDofs,
	 solver_t::callback_t additionalCallback);

	void addConstraints
	(bool enableFreezeFrame,
	 bool enableVelocity,
	 bool enableFeetPositions,
	 bool enableCollision,
	 bool enableTorque,
	 bool enableZmp);

      private:
	static log4cxx::LoggerPtr logger;

	cnoid::BodyPtr robot_;

	/// \name Problem configuration
	/// \{

	/// \brief number of frames
	std::size_t nFrames_;
	/// \brief delta t used for time synchronization (vector
	///        interpolation only)
	value_type dt_;
	/// \brief number of spline nodes (spline only)
	value_type nNodes_;
	/// \brief minimum trajectory time
	value_type tmin_;
	/// \brief maximum trajectory time
	value_type tmax_;
	/// \brief the id of the dof we move.
	std::size_t dofId_;
	/// \brief dof initial position
	value_type init_;
	/// \brief dof goal position
	value_type goal_;
	/// \brief total number of dofs
	std::size_t nDofs_;

	MinimumJerkTrajectory<EigenMatrixDense>::discreteInterval_t
	interval_;

	/// \}

	/// \brief Cost function
	DifferentiableFunctionShPtr_t cost_;

	/// \name Constraints
	/// \{

	/// \brief Linear function freezing first frame.
	LinearFunctionShPtr_t freeze_;
	/// \brief Joint velocity
	LinearFunctionShPtr_t velocity_;
	/// \brief Body constraints
	/// 0 is left foot and 1 is right foot
	std::vector<DifferentiableFunctionShPtr_t> positions_;
	/// \brief Torque
	DifferentiableFunctionShPtr_t torque_;
	/// \brief ZMP
	DifferentiableFunctionShPtr_t zmp_;

	/// \}

	/// \brief Trajectory used by constraints.
	boost::shared_ptr<Trajectory<3> >
	trajectoryConstraints_;

	/// \brief Optimization problem.
	ProblemShPtr_t problem_;

	GenericSolver::result_t result_;

	std::string solverName_;

	/// \brief Additional callback on top of usual logging.
	solver_t::callback_t additionalCallback_;

	/// \brief Whether each DOF must be considered or not
	std::vector<bool> enabledDofs_;
      };

    } // end of namespace problem.
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_PROBLEM_JOINT_HH
