#include <boost/make_shared.hpp>

#include <roboptim/core/solver-factory.hh>

#include "roboptim/retargeting/retarget.hh"

namespace roboptim
{
  namespace retargeting
  {
    log4cxx::LoggerPtr Retarget::logger
    (log4cxx::Logger::getLogger("roboptim.retargeting.Retarget"));

    Retarget::Retarget (const std::string& initialTrajectory,
			const std::string& character,
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
	solverName_ (solverName)
    {
      std::vector<DifferentiableFunctionShPtr_t> costs;
      costs.push_back (costLaplacian_);
      costs.push_back (costAcceleration_);
      cost_ = boost::make_shared<Sum> (costs);
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
	  for (unsigned i = 0; i < boneLengths_.size (); ++i)
	    problem_->addConstraint
	      (boost::static_pointer_cast<LinearFunction> (boneLengths_[i]),
	       intervals,
	       scales);
	}
      else
	LOG4CXX_INFO (logger, "bone lengths constraints disabled");

      // -- Position
      if (enablePosition)
	for (unsigned i = 0; i < positions_.size (); ++i)
	  problem_->addConstraint (positions_[i],
				   roboptim::Function::makeInterval (0., 0.));

      // -- Collision
      if (enableCollision)
	for (unsigned i = 0; i < collisions_.size (); ++i)
	  problem_->addConstraint (collisions_[i],
				   roboptim::Function::makeInterval (0., 0.));

      // -- Torque
      if (enableTorque)
	for (unsigned i = 0; i < torques_.size (); ++i)
	  problem_->addConstraint (torques_[i],
				   roboptim::Function::makeInterval (0., 0.));
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
      solver.parameters ()["max-iterations"].value = 10;
      solver.parameters ()["ipopt.output_file"].value =
	"/tmp/ipopt.log";

      LOG4CXX_INFO (logger, "Solver:\n" << solver);
      LOG4CXX_DEBUG(logger, "start solving...");
      solver.solve ();
      LOG4CXX_DEBUG(logger, "problem solved.");
      result_ = solver.minimum ();
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
