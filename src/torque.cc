#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/torque.hh"

#include <metapod/models/simple_humanoid/simple_humanoid.hh>

// FIXME: please vomit here.
#include <metapod/models/simple_humanoid/simple_humanoid.cc>

#include <metapod/tools/print.hh>
#include <metapod/algos/rnea.hh>

// Define which robot to use.
typedef metapod::simple_humanoid robot_t;

namespace roboptim
{
  namespace retargeting
  {
    Torque::Torque
    (AnimatedInteractionMeshShPtr_t animatedMesh,
     AnimatedInteractionMeshShPtr_t animatedMeshLocal) throw ()
      : roboptim::GenericDifferentiableFunction<EigenMatrixSparse>
	(static_cast<size_type> (animatedMesh->optimizationVectorSize ()),
	 animatedMeshLocal_->numFrames () * robot_t::NBDOF, "torque"),
	animatedMesh_ (animatedMesh),
	animatedMeshLocal_ (animatedMeshLocal)
    {}

    Torque::~Torque () throw ()
    {}

    void
    Torque::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      animatedMeshLocal_->state () = x;
      animatedMeshLocal_->computeVertexWeights();

      robot_t robot;

      std::vector<robot_t::confVector> q (animatedMeshLocal_->numFrames ());
      std::vector<robot_t::confVector> dq (animatedMeshLocal_->numFrames ());
      std::vector<robot_t::confVector> ddq (animatedMeshLocal_->numFrames ());

      // Compute q from segment positions.
      for (unsigned frameId = 0; frameId < animatedMeshLocal_->numFrames ();
	   ++frameId)
	{
	for (unsigned dofId = 0; dofId < robot_t::NBDOF; ++dofId)
	  {
	    // 1. which segment are related to this dof?
	    AnimatedInteractionMesh::vertex_descriptor_t segment1  = 0;
	    AnimatedInteractionMesh::vertex_descriptor_t segment2  = 0;

	    // 2. get segment positions
	    const Vertex::position_t& segment1Position =
	      animatedMeshLocal_->graph ()[segment1].positions[frameId];
	    const Vertex::position_t& segment2Position =
	      animatedMeshLocal_->graph ()[segment2].positions[frameId];

	    // 3. identify articular value
	    q[frameId][dofId] =
	      std::asin
	      (segment1Position.cross (segment2Position)[0]
	       / segment1Position.norm ()
	       / segment2Position.norm ());
	  }
	}

      // Fill dq with frameId = 0 to n - 1.
      for (unsigned frameId = 0; frameId < animatedMeshLocal_->numFrames () - 1;
	   ++frameId)
	{
	  for (unsigned dofId = 0; dofId < robot_t::NBDOF; ++dofId)
	    dq[frameId][dofId] =
	      (q[frameId][dofId] - q[frameId + 1][dofId])
	      / animatedMeshLocal_->framerate ();
	  dq[frameId][animatedMeshLocal_->numFrames () - 1] = 0.;
	}

      // Fill ddq with frameId = 1 to n - 1
      for (unsigned frameId = 1; frameId < animatedMeshLocal_->numFrames () - 1;
	   ++frameId)
	{
	  for (unsigned dofId = 0; dofId < robot_t::NBDOF; ++dofId)
	    ddq[frameId][dofId] =
	      (q[frameId + 1][dofId]
	     - 2 * q[frameId][dofId]
	       - q[frameId - 1][dofId])
	      / (animatedMeshLocal_->framerate () 
		 * animatedMeshLocal_->framerate ());
	  dq[frameId][0] = 0.;
	  dq[frameId][animatedMeshLocal_->numFrames () - 1] = 0.;
	}

      robot_t::confVector torques;
      for (unsigned frameId = 1; frameId < animatedMeshLocal_->numFrames () - 1;
	   ++frameId)
	{
	  metapod::rnea<robot_t, true>::run
	    (robot, q[frameId], dq[frameId], ddq[frameId]);
	  metapod::getTorques (robot, torques);

	  result.segment
	    (frameId * robot_t::NBDOF, robot_t::NBDOF) = torques;
	}
    }

    void
    Torque::impl_gradient
    (gradient_t& gradient,
     const argument_t& x,
     size_type i)
      const throw ()
    {
      roboptim::GenericFiniteDifferenceGradient<
	EigenMatrixSparse,
	finiteDifferenceGradientPolicies::Simple<EigenMatrixSparse> >
	fdg (*this);
      fdg.gradient (gradient, x, i);
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
