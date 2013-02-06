#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/bone-length.hh"

namespace roboptim
{
  namespace retargeting
  {
    BoneLength::BoneLength
    (AnimatedInteractionMeshShPtr_t animatedMesh,
     AnimatedInteractionMeshShPtr_t animatedMeshLocal,
     unsigned frameId,
     AnimatedInteractionMesh::edge_descriptor_t edgeId) throw ()
      : roboptim::LinearFunction
	(animatedMesh->optimizationVectorSize (),
	 1, ""),
	animatedMesh_ (animatedMesh),
	animatedMeshLocal_ (animatedMeshLocal),
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
      AnimatedInteractionMesh::vertex_descriptor_t source =
	boost::source (edgeId_, animatedMesh_->graph ());
      AnimatedInteractionMesh::vertex_descriptor_t target =
	boost::target (edgeId_, animatedMesh_->graph ());

      const Vertex& sourceVertex = animatedMesh_->graph ()[source];
      const Vertex& targetVertex = animatedMesh_->graph ()[target];
      const Edge& edge = animatedMesh_->graph ()[edgeId_];

      const unsigned& nVertices = animatedMesh_->numVertices ();

      animatedMeshLocal_->state () = x;
      animatedMeshLocal_->computeVertexWeights();

      const Vertex& sourceVertexNew =
      	animatedMeshLocal_->graph ()[source];
      const Vertex& targetVertexNew =
      	animatedMeshLocal_->graph ()[target];

      result[0] =
      	(targetVertex.positions[frameId_] - sourceVertex.positions[frameId_]
	 ).squaredNorm ();
      result[0] -= edge.scale *
      	(targetVertexNew.positions[frameId_]
	 - sourceVertexNew.positions[frameId_]).squaredNorm ();
    }

    void
    BoneLength::impl_gradient
    (gradient_t& gradient,
     const argument_t& x ,
     size_type i)
      const throw ()
    {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

      roboptim::FiniteDifferenceGradient<
	finiteDifferenceGradientPolicies::Simple>
	fdg (*this);
      fdg.gradient (gradient, x, i);
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
