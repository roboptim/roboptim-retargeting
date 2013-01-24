#include <boost/make_shared.hpp>

#include "roboptim/retargeting/retarget.hh"

namespace roboptim
{
  namespace retargeting
  {
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
	problem_ ()
    {
      std::vector<DifferentiableFunctionShPtr_t> costs;
      costs.push_back (costLaplacian_);
      costs.push_back (costAcceleration_);
      cost_ = boost::make_shared<Sum> (costs);
      problem_ = boost::make_shared<problem_t> (*cost_);

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
		boost::make_shared<BoneLength> ();
	      boneLengths_.push_back (boneLengthConstraint);
	    }

      // Add constraints to problem.

      // -- Bone length
      for (unsigned i = 0; i < boneLengths_.size (); ++i)
	problem_->addConstraint (boneLengths_[i],
				 roboptim::Function::makeInterval (0., 0.));

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
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
