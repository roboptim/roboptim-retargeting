#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_RUNTIME_NO_MALLOC
#include <rbdl/rbdl.h>

#include <boost/make_shared.hpp>

#include <urdf_parser/urdf_parser.h>

#include <roboptim/core/solver-factory.hh>

#include "roboptim/retargeting/inverse-kinematics.hh"
#include "roboptim/retargeting/retarget.hh"

namespace roboptim
{
  namespace retargeting
  {
    log4cxx::LoggerPtr Retarget::logger
    (log4cxx::Logger::getLogger("roboptim.retargeting.Retarget"));

    Retarget::Retarget (const std::string& initialTrajectory,
			const std::string& character,
			const std::string& model,
			bool enableBoneLength,
			bool enablePosition,
			bool enableCollision,
			bool enableTorque,
			const std::string& solverName)
      : animatedMesh_
	(AnimatedInteractionMesh::loadAnimatedMesh
	 (initialTrajectory, character)),
	costLaplacian_
	(boost::make_shared<LaplacianDeformationEnergy> (animatedMesh_)),
	costAcceleration_
	(boost::make_shared<AccelerationEnergy> (animatedMesh_)),
	cost_ (),
	problem_ (),
	result_ (),
	solverName_ (solverName),
	model_ (urdf::parseURDF (model))
    {
      std::vector<DifferentiableFunctionShPtr_t> costs;
      costs.push_back (costLaplacian_);
      costs.push_back (costAcceleration_);
      cost_ = boost::make_shared<Sum<EigenMatrixSparse> > (costs);
      problem_ = boost::make_shared<problem_t> (*cost_);

      // Use loaded animated interaction mesh to provide starting
      // point.
      problem_->startingPoint () =
	animatedMesh_->makeOptimizationVector ();

      // Create bone lengths constraints,
      // one per edge and per frame if the scale is different from 1.

      AnimatedInteractionMeshShPtr_t animatedMeshLocal =
	AnimatedInteractionMesh::makeFromOptimizationVariables
	(animatedMesh_->state (), animatedMesh_);

      AnimatedInteractionMesh::edge_iterator_t edgeIt;
      AnimatedInteractionMesh::edge_iterator_t edgeEnd;
      boost::tie (edgeIt, edgeEnd) = boost::edges (animatedMesh_->graph ());
      for (; edgeIt != edgeEnd; ++edgeIt)
	if (animatedMesh_->graph ()[*edgeIt].scale != 1.)
	  {
	    BoneLengthShPtr_t boneLengthConstraint =
	      boost::make_shared<BoneLength>
	      (animatedMesh_, animatedMeshLocal, *edgeIt);
	    boneLengths_.push_back (boneLengthConstraint);
	  }

      // Create position constraints.
      //FIXME:

      // Create torque constraints.
      std::string modelOpenHrp
	("/home/moulard/HRP4C-release/HRP4Cg2main.wrl");
      boost::shared_ptr<InverseKinematics> ik =
	InverseKinematics::create (modelOpenHrp);
      torque_ = boost::make_shared<Torque>
	(animatedMesh_, animatedMeshLocal, ik);

      // Add constraints to problem.

      // -- Bone length
      if (enableBoneLength)
	{
	  LOG4CXX_INFO (logger, "bone lengths constraints enabled");

	  Function::intervals_t intervals;
	  problem_t::scales_t scales;
	  for (unsigned i = 0; i < animatedMesh_->numFrames (); ++i)
	    {
	      intervals.push_back (roboptim::Function::makeInterval (0., 0.));
	      scales.push_back (1.);
	    }
	  for (unsigned i = 0; i < 1; ++i)
	    problem_->addConstraint
	      (boost::static_pointer_cast
	       <GenericLinearFunction<EigenMatrixSparse> >
	       (boneLengths_[i]),
	       intervals,
	       scales);
	}
      else
	LOG4CXX_INFO (logger, "bone lengths constraints disabled");

      // -- Position
      if (enablePosition)
	for (unsigned i = 0; i < positions_.size (); ++i)
	  {
	    LOG4CXX_INFO (logger, "position constraints enabled");

	    Function::intervals_t intervals;
	    problem_t::scales_t scales;
	    for (unsigned j = 0; j < positions_[i]->outputSize (); ++j)
	      {
		intervals.push_back (roboptim::Function::makeInterval (0., 0.));
		scales.push_back (1.);
	      }

	    problem_->addConstraint
	      (boost::static_pointer_cast
	       <GenericLinearFunction<EigenMatrixSparse> > (positions_[i]),
	       intervals, scales);
	  }

      // -- Collision
      if (enableCollision)
	for (unsigned i = 0; i < collisions_.size (); ++i)
	  problem_->addConstraint
	    (collisions_[i],
	     roboptim::Function::makeInterval (0., 0.));

      // -- Torque
      if (enableTorque)
	{
	  LOG4CXX_INFO (logger, "torque constraints enabled");

	  Function::intervals_t intervals;
	  problem_t::scales_t scales;
	  for (unsigned i = 0; i < torque_->outputSize (); ++i)
	    {
	      intervals.push_back (roboptim::Function::makeInterval (0., 0.));
	      scales.push_back (1.);
	    }

	  problem_->addConstraint
	    (boost::static_pointer_cast
	     <GenericDifferentiableFunction<EigenMatrixSparse> >
	     (torque_), intervals, scales);
	}
    }

    Retarget::~Retarget ()
    {
    }

    void
    Retarget::solve ()
    {
      roboptim::SolverFactory<solver_t> factory (solverName_, *problem_);
      solver_t& solver = factory ();

      // Set solver parameters.
      solver.parameters ()["max-iterations"].value = 100;
      solver.parameters ()["ipopt.output_file"].value =
	"/tmp/ipopt.log";
      solver.parameters ()["ipopt.expect_infeasible_problem"].value = "yes";

      LOG4CXX_INFO (logger, "Solver:\n" << solver);
      LOG4CXX_DEBUG(logger, "start solving...");
      solver.solve ();
      LOG4CXX_DEBUG(logger, "problem solved.");
      result_ = solver.minimum ();
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
