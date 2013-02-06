#include <boost/format.hpp>
#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/bone-length.hh"

namespace roboptim
{
  namespace retargeting
  {
    BoneLength::BoneLength
    (AnimatedInteractionMeshShPtr_t animatedMesh,
     AnimatedInteractionMeshShPtr_t animatedMeshLocal,
     AnimatedInteractionMesh::edge_descriptor_t edgeId) throw ()
      : roboptim::LinearFunction
	(animatedMesh->optimizationVectorSize (),
	 animatedMesh->numFrames (),
	 (boost::format ("bone length (edge id = %1%") % edgeId).str ()),
	animatedMesh_ (animatedMesh),
	animatedMeshLocal_ (animatedMeshLocal),
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

      animatedMeshLocal_->state () = x;
      animatedMeshLocal_->computeVertexWeights();

      const Vertex& sourceVertexNew =
      	animatedMeshLocal_->graph ()[source];
      const Vertex& targetVertexNew =
      	animatedMeshLocal_->graph ()[target];

      for (unsigned i = 0; i < animatedMesh_->numFrames (); ++i)
	{
	  result[i] =
	    (targetVertex.positions[i] - sourceVertex.positions[i]
	     ).squaredNorm ();
	  result[i] -= edge.scale *
	    (targetVertexNew.positions[i]
	     - sourceVertexNew.positions[i]).squaredNorm ();
	}
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
