#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/zmp.hh"

#include "model/hrp4g2.hh"

#include <metapod/tools/print.hh>
#include <metapod/algos/rnea.hh>

// Define which robot to use.
typedef metapod::hrp4g2 robot_t;

namespace roboptim
{
  namespace retargeting
  {
    ZMP::ZMP
    (AnimatedInteractionMeshShPtr_t animatedMesh,
     AnimatedInteractionMeshShPtr_t animatedMeshLocal) throw ()
      : roboptim::GenericDifferentiableFunction<EigenMatrixSparse>
	(static_cast<size_type> (animatedMesh->optimizationVectorSize ()),
	 animatedMeshLocal_->numFrames () * 2, "ZMP"),
	animatedMesh_ (animatedMesh),
	animatedMeshLocal_ (animatedMeshLocal)
    {}

    ZMP::~ZMP () throw ()
    {}

    // see https://github.com/laas/metapod/issues/63
    void
    ZMP::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      animatedMeshLocal_->state () = x;
      animatedMeshLocal_->computeVertexWeights();

      robot_t robot;

      std::vector<robot_t::confVector> q (animatedMeshLocal_->numFrames ());
      std::vector<robot_t::confVector> dq (animatedMeshLocal_->numFrames ());
      std::vector<robot_t::confVector> ddq (animatedMeshLocal_->numFrames ());

      //FIXME: fill q.

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
      for (unsigned frameId = 0; frameId < animatedMeshLocal_->numFrames ();
	   ++frameId)
	{
	  metapod::rnea<robot_t, true>::run
	    (robot, q[frameId], dq[frameId], ddq[frameId]);
	  metapod::getTorques (robot, torques);

	  // Express root spatial resultant force in world frame.
	  // BODY is the floating base link, WAIST is the floating joint.
	  metapod::Spatial::Force af =
	    boost::fusion::at_c<0>
	    (robot.nodes).body.iX0.applyInv
	    (boost::fusion::at_c<0>
	     (robot.nodes).joint.f);

	  if (af.f()[2])
	    {
	      result[frameId * 2 + 0] = 0.;
	      result[frameId * 2 + 1] = 0.;
	      continue;
	    }

	  // Compute ZMP
	  result[frameId * 2 + 0] = - af.n()[1] / af.f()[2];
	  result[frameId * 2 + 1] =   af.n()[0] / af.f()[2];
	}
    }

    void
    ZMP::impl_gradient
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
