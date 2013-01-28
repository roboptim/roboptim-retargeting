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
			const std::string& character)
      : animatedMesh_
	(AnimatedInteractionMesh::loadAnimatedMesh
	 (initialTrajectory, character)),
	costLaplacian_
	(boost::make_shared<LaplacianDeformationEnergy> (animatedMesh_)),
	costAcceleration_
	(boost::make_shared<AccelerationEnergy> (animatedMesh_)),
	cost_ (),
	problem_ (),
	result_ ()
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
      InteractionMesh::edge_iterator_t edgeIt;
      InteractionMesh::edge_iterator_t edgeEnd;
      boost::tie (edgeIt, edgeEnd) =
	boost::edges (animatedMesh_->meshes ()[0]->graph ());
      for (; edgeIt != edgeEnd; ++edgeIt)
	if (animatedMesh_->meshes ()[0]->graph ()[*edgeIt].scale != 1.)
	  for (unsigned i = 0; i < animatedMesh_->meshes ().size (); ++i)
	    {
	      BoneLengthShPtr_t boneLengthConstraint =
		boost::make_shared<BoneLength> (animatedMesh_, i, *edgeIt);
	      boneLengths_.push_back (boneLengthConstraint);
	    }

      // Add constraints to problem.

      // -- Bone length
      // for (unsigned i = 0; i < boneLengths_.size (); ++i)
      // 	problem_->addConstraint (boneLengths_[i],
      // 				 roboptim::Function::makeInterval (0., 0.));

      // -- Position
      for (unsigned i = 0; i < positions_.size (); ++i)
	problem_->addConstraint (positions_[i],
				 roboptim::Function::makeInterval (0., 0.));

      // -- Collision
      for (unsigned i = 0; i < collisions_.size (); ++i)
	problem_->addConstraint (collisions_[i],
				 roboptim::Function::makeInterval (0., 0.));

      // -- Torque
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
      roboptim::SolverFactory<solver_t> factory ("ipopt", *problem_);
      solver_t& solver = factory ();

      // Set solver parameters.
      solver.parameters ()["max-iterations"].value = 1;
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
