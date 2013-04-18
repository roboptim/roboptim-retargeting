#ifndef ROBOPTIM_RETARGETING_RETARGET_HH
# define ROBOPTIM_RETARGETING_RETARGET_HH
# include <string>

# include <boost/shared_ptr.hpp>

# include <roboptim/core/differentiable-function.hh>
# include <roboptim/core/generic-solver.hh>
# include <roboptim/core/linear-function.hh>
# include <roboptim/core/problem.hh>
# include <roboptim/core/solver.hh>
# include <roboptim/core/sum-of-c1-squares.hh>

# include <roboptim/retargeting/acceleration-energy.hh>
# include <roboptim/retargeting/animated-interaction-mesh.hh>
# include <roboptim/retargeting/laplacian-deformation-energy.hh>
# include <roboptim/retargeting/sum.hh>

# include <roboptim/retargeting/bone-length.hh>
# include <roboptim/retargeting/position.hh>
# include <roboptim/retargeting/collision.hh>
# include <roboptim/retargeting/torque.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Reshape motion to fit a robot model.
    ///
    /// Build the optimization problem and provide a method #solve ()
    /// which looks for an optimal solution.
    class Retarget
    {
    public:
      typedef boost::shared_ptr<
	typename roboptim::GenericDifferentiableFunction<
	  EigenMatrixSparse> >
      DifferentiableFunctionShPtr_t;
      typedef boost::shared_ptr<Sum<EigenMatrixSparse> > SumShPtr_t;

      typedef roboptim::Problem<
      GenericDifferentiableFunction<EigenMatrixSparse>,
      boost::mpl::vector<
	GenericLinearFunction<EigenMatrixSparse>,
	GenericDifferentiableFunction<EigenMatrixSparse>
	> >
      problem_t;
      typedef roboptim::Solver<
	GenericDifferentiableFunction<EigenMatrixSparse>,
	boost::mpl::vector<
	  GenericLinearFunction<EigenMatrixSparse>,
	  GenericDifferentiableFunction<EigenMatrixSparse>
	  > >
      solver_t;
      typedef boost::shared_ptr<problem_t>
      problemShPtr_t;

      explicit Retarget
      (const std::string& initialTrajectory,
       const std::string& character,
       bool enableBoneLength,
       bool enablePosition,
       bool enableCollision,
       bool enableTorque,
       const std::string& solverName);
      virtual ~Retarget ();

      /// \brief Solve the underlying optimization problem.
      void solve ();

      AnimatedInteractionMeshShPtr_t animatedMesh ()
      {
	return animatedMesh_;
      }

      const LaplacianDeformationEnergyShPtr_t& costLaplacian () const
      {
	return costLaplacian_;
      }

      LaplacianDeformationEnergyShPtr_t& costLaplacian ()
      {
	return costLaplacian_;
      }

      const AccelerationEnergyShPtr_t& costAcceleration () const
      {
	return costAcceleration_;
      }

      AccelerationEnergyShPtr_t& costAcceleration ()
      {
	return costAcceleration_;
      }

      const SumShPtr_t& cost () const
      {
	return cost_;
      }

      SumShPtr_t& cost ()
      {
	return cost_;
      }

      problemShPtr_t& problem ()
      {
	return problem_;
      }

      const problemShPtr_t& problem () const
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

    private:
      static log4cxx::LoggerPtr logger;

      AnimatedInteractionMeshShPtr_t animatedMesh_;
      LaplacianDeformationEnergyShPtr_t costLaplacian_;
      AccelerationEnergyShPtr_t costAcceleration_;
      SumShPtr_t cost_;
      problemShPtr_t problem_;

      std::vector<BoneLengthShPtr_t> boneLengths_;
      std::vector<PositionShPtr_t> positions_;
      std::vector<CollisionShPtr_t> collisions_;
      TorqueShPtr_t torque_;

      GenericSolver::result_t result_;

      std::string solverName_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_RETARGETING_HH
