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
			cnoid::MarkerIMeshPtr markerIMesh,
			bool enableBoneLength,
			bool enablePosition,
			bool enableCollision,
			bool enableTorque,
			bool enableZmp,
			const std::string& solverName,
			cnoid::MarkerMotionItemPtr markerMotionItem)
      : characterInfos (new std::vector<CharacterInfo> ()),
	mesh (markerIMesh),
	animatedMesh_
	(AnimatedInteractionMesh::loadAnimatedMesh
	 (markerMotion, character)),
	costLaplacian_ (),
	costAcceleration_
	(AccelerationEnergyShPtr_t
	 (new AccelerationEnergy (animatedMesh_))),
	cost_ (),
	problem_ (),
	boneLength_ (),
	positions_ (),
	collisions_ (),
	torque_ (),
	zmp_ (),
	result_ (),
	solverName_ (solverName)
    {
      markerIMesh->addMotion (markerMotion, character);
      markerIMesh->update ();

      setCharacterPair (0, character, character);

      costLaplacian_ = LaplacianDeformationEnergyShPtr_t
	(new LaplacianDeformationEnergy
	 (animatedMesh_, markerMotion, character, markerIMesh,
	  characterInfos));

      std::vector<DifferentiableFunctionShPtr_t> costs;
      costs.push_back (costLaplacian_);
      //costs.push_back (costAcceleration_);
      cost_ = boost::make_shared<Sum<EigenMatrixSparse> > (costs);
      problem_ = boost::make_shared<problem_t> (*cost_);

      // Use loaded animated interaction mesh to provide starting
      // point.
      // problem_->startingPoint () =
      // 	animatedMesh_->makeOptimizationVector ();
      // Set the starting point.
      std::cout << "PROBLEM INPUT SIZE: " << problem_->function ().inputSize () << "\n";
      cnoid::VectorXd x (problem_->function ().inputSize ());

      for (int frameId = 0;
	   frameId < markerMotion->numFrames (); ++frameId)
	{
	  cnoid::MarkerMotion::Frame frame = markerMotion->frame (frameId);
	  for (int partId = 0;
	       partId < markerMotion->numParts (); ++partId)
		 x.segment<3>
		 (frameId * markerMotion->numParts () * 3 + partId * 3) =
		   frame[partId];
	}

      problem_->startingPoint () = x;
      std::cout << (*problem_->startingPoint ()) << std::endl;



      AnimatedInteractionMeshShPtr_t animatedMeshLocal =
	AnimatedInteractionMesh::makeFromOptimizationVariables
	(animatedMesh_->state (), animatedMesh_);


      // Create bone lengths constraint,
      int numAllBones = costLaplacian_->numAllBones;
      std::cout << "NUM ALL BONES: " << numAllBones << std::endl;
      int numFrames = mesh->numFrames ();
      AnimatedInteractionMesh::edge_descriptor_t it;
      boneLength_ = BoneLengthShPtr_t
	(new BoneLength
	 (markerIMesh, characterInfos, numAllBones));

      // Create position constraints.
      int leftAnkle = markerMotion->markerIndex("LeftAnkle");
      const cnoid::Vector3& p = markerMotionItem->currentMarkerPosition(leftAnkle);
      positions_.push_back
	(PositionShPtr_t (new Position (mesh, 0, leftAnkle, p, false, 1., numAllBones)));

      int rightAnkle = markerMotion->markerIndex("RightAnkle");
      const cnoid::Vector3& p2 = markerMotionItem->currentMarkerPosition(rightAnkle);
      positions_.push_back
	(PositionShPtr_t (new Position (mesh, 0, rightAnkle, p2, false, 1., numAllBones)));


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
	  for (unsigned i = 0; i < numAllBones * numFrames; ++i)
	    {
	      intervals.push_back (roboptim::Function::makeInterval (0., 0.));
	      scales.push_back (1.);
	    }
	  problem_->addConstraint
	    (boost::static_pointer_cast
	     <GenericNumericLinearFunction<EigenMatrixSparse> >
	     (boneLength_),
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
	       <GenericNumericLinearFunction<EigenMatrixSparse> > (positions_[i]),
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
	      intervals.push_back
		(roboptim::Function::makeInterval
		 (torqueConstraint ()->torqueLimits_[i].first,
		  torqueConstraint ()->torqueLimits_[i].second));
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
	      intervals.push_back (roboptim::Function::makeInterval (-.1, .1));
	      scales.push_back (1.);
	    }

	  problem_->addConstraint
	    (boost::static_pointer_cast
	     <GenericDifferentiableFunction<EigenMatrixSparse> >
	     (zmp_), intervals, scales);
	}
    }

    Retarget::~Retarget ()
    {
    }

    void
    Retarget::setCharacterPair
    (int motionIndex, cnoid::CharacterPtr org, cnoid::CharacterPtr goal)
    {
      if(mesh && motionIndex < mesh->numMotions())
	{
	  if(motionIndex >= characterInfos->size())
	    {
	      characterInfos->resize(motionIndex + 1);
	    }
	  CharacterInfo& chara = (*characterInfos)[motionIndex];
	  if(!chara.org)
	    {
	      chara.org = org;
	      chara.goal = goal;
	    }
	}
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
      solver.parameters ()["ipopt.nlp_scaling_method"].value = "none";

      LOG4CXX_INFO (logger, "Solver:\n" << solver);
      LOG4CXX_DEBUG(logger, "start solving...");
      solver.solve ();
      LOG4CXX_DEBUG(logger, "problem solved.");
      result_ = solver.minimum ();
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
