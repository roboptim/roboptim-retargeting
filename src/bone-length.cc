#include "roboptim/retargeting/bone-length.hh"

namespace roboptim
{
  namespace retargeting
  {
    BoneLength::BoneLength
    (AnimatedInteractionMeshShPtr_t animatedMesh,
     unsigned frameId,
     InteractionMesh::edge_descriptor_t edgeId) throw ()
      : roboptim::DifferentiableFunction
	(animatedMesh->optimizationVectorSize (),
	 1, ""),
	animatedMesh_ (animatedMesh),
	frameId_ (frameId),
	edgeId_ (edgeId)
    {}

    BoneLength::~BoneLength () throw ()
    {}

    void
    BoneLength::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      result[1] = 0.;
    }

    void
    BoneLength::impl_gradient
    (gradient_t&,
     const argument_t&,
     size_type)
      const throw ()
    {
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
