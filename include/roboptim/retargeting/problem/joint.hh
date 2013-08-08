#ifndef ROBOPTIM_RETARGETING_PROBLEM_JOINT_HH
# define ROBOPTIM_RETARGETING_PROBLEM_JOINT_HH
# include <string>

# include <boost/shared_ptr.hpp>

# include <roboptim/core/differentiable-function.hh>
# include <roboptim/core/generic-solver.hh>
# include <roboptim/core/linear-function.hh>
# include <roboptim/core/problem.hh>
# include <roboptim/core/solver.hh>

# include <roboptim/retargeting/acceleration-energy.hh>
# include <roboptim/retargeting/animated-interaction-mesh.hh>
# include <roboptim/retargeting/laplacian-deformation-energy.hh>

# include <roboptim/retargeting/bone-length.hh>
# include <roboptim/retargeting/position.hh>
# include <roboptim/retargeting/collision.hh>
# include <roboptim/retargeting/torque.hh>
# include <roboptim/retargeting/zmp.hh>

# include <cnoid/Body>
# include <cnoid/src/MocapPlugin/MarkerMotion.h>
# include <cnoid/src/MocapPlugin/MarkerMotionItem.h>

namespace roboptim
{
  namespace retargeting
  {
    namespace problem
    {
      /// \brief Define the optimization problem on robot joint space.
      ///
      /// Build the optimization problem and provide a method #solve ()
      /// which looks for an optimal solution.
      class Joint
      {
      public:
	typedef boost::shared_ptr<
	roboptim::GenericDifferentiableFunction<
	  EigenMatrixSparse> >
	DifferentiableFunctionShPtr_t;

	typedef roboptim::Solver<

	  GenericDifferentiableFunction<EigenMatrixSparse>,
	  boost::mpl::vector<
	    GenericNumericLinearFunction<EigenMatrixSparse>,
	    GenericDifferentiableFunction<EigenMatrixSparse> >

	  /*
	    GenericDifferentiableFunction<EigenMatrixSparse>,
	    boost::mpl::vector<
	    GenericLinearFunction<EigenMatrixSparse>,
	    GenericDifferentiableFunction<EigenMatrixSparse>
	    >
	  */
	  >
	solver_t;
    typedef solver_t::problem_t problem_t;

    typedef boost::shared_ptr<problem_t>
    problemShPtr_t;

    explicit Joint
    (cnoid::MarkerMotionPtr markerMotion,
     cnoid::CharacterPtr character,
     cnoid::BodyPtr body,
     cnoid::MarkerIMeshPtr jointIMesh,
     bool enableBoneLength,
     bool enablePosition,
     bool enableCollision,
     bool enableTorque,
     bool enableZmp,
     const std::string& solverName,
     cnoid::MarkerMotionItemPtr markerMotionItem);
    virtual ~Joint ();

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

    const DifferentiableFunctionShPtr_t& cost () const
    {
      return cost_;
    }

    DifferentiableFunctionShPtr_t& cost ()
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

    TorqueShPtr_t torqueConstraint ()
    {
      return torque_;
    }

    void setCharacterPair
    (int motionIndex, cnoid::CharacterPtr org, cnoid::CharacterPtr goal);


  private:
    static log4cxx::LoggerPtr logger;

    boost::shared_ptr<std::vector<CharacterInfo> > characterInfos;
    cnoid::MarkerIMeshPtr mesh;

    AnimatedInteractionMeshShPtr_t animatedMesh_;
    LaplacianDeformationEnergyShPtr_t costLaplacian_;
    AccelerationEnergyShPtr_t costAcceleration_;
    DifferentiableFunctionShPtr_t cost_;
    problemShPtr_t problem_;

    BoneLengthShPtr_t boneLength_;
    std::vector<PositionShPtr_t> positions_;
    std::vector<CollisionShPtr_t> collisions_;
    TorqueShPtr_t torque_;
    ZmpShPtr_t zmp_;

    GenericSolver::result_t result_;

    std::string solverName_;
  };

} // end of namespace problem.
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_PROBLEM_JOINT_HH
