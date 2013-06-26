#include <boost/make_shared.hpp>
#include <roboptim/core/solver-factory.hh>

#include <cnoid/Body>

#include "roboptim/retargeting/retarget.hh"

namespace roboptim
{
  namespace retargeting
  {
    log4cxx::LoggerPtr Retarget::logger
    (log4cxx::Logger::getLogger("roboptim.retargeting.Retarget"));

    Retarget::Retarget (cnoid::MarkerMotionPtr markerMotion,
			cnoid::CharacterPtr character,
			cnoid::BodyPtr body,
			bool enableBoneLength,
			bool enablePosition,
			bool enableCollision,
			bool enableTorque,
			bool enableZmp,
			const std::string& solverName)
      : animatedMesh_
	(AnimatedInteractionMesh::loadAnimatedMesh
	 (markerMotion, character)),
	costLaplacian_
	(boost::make_shared<LaplacianDeformationEnergy> (animatedMesh_, markerMotion, character)),
	costAcceleration_
	(boost::make_shared<AccelerationEnergy> (animatedMesh_)),
	cost_ (),
	problem_ (),
	boneLengths_ (),
	positions_ (),
	collisions_ (),
	torque_ (),
	zmp_ (),
	result_ (),
	solverName_ (solverName)
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
      torque_ = boost::make_shared<Torque>
	(animatedMesh_, animatedMeshLocal, body, markerMotion);

      zmp_ = boost::make_shared<ZMP>
	(animatedMesh_, animatedMeshLocal, body, markerMotion);

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

      // -- ZMP
      if (enableZmp)
	{
	  LOG4CXX_INFO (logger, "zmp constraints enabled");

	  Function::intervals_t intervals;
	  problem_t::scales_t scales;
	  for (unsigned i = 0; i < zmp_->outputSize (); ++i)
	    {
	      //FIXME:
	      intervals.push_back (roboptim::Function::makeInterval (-1., 1.));
	      scales.push_back (1.);
	    }

	  problem_->addConstraint
	    (boost::static_pointer_cast
	     <GenericDifferentiableFunction<EigenMatrixSparse> >
	     (zmp_), intervals, scales);
	}

      // Set the starting point.
      cnoid::VectorXd x (problem_->function ().inputSize ());

      for (int frameId = 0;
	   frameId < markerMotion->numFrames (); ++frameId)
	{
	  cnoid::MarkerMotion::Frame frame = markerMotion->frame (frameId);
	  int idx = 0;
	  for (int partId = 0;
	       partId < markerMotion->numParts (); ++partId)
		 x.segment<3>
		 (frameId * markerMotion->numParts () * 3 + partId * 3) = frame[idx++];
	}

      problem_->startingPoint () = x;
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
      solver.parameters ()["ipopt.expect_infeasible_problem"].value = "yes";

      LOG4CXX_INFO (logger, "Solver:\n" << solver);
      LOG4CXX_DEBUG(logger, "start solving...");
      solver.solve ();
      LOG4CXX_DEBUG(logger, "problem solved.");
      result_ = solver.minimum ();
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
