#include <roboptim/core/finite-difference-gradient.hh>
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
      InteractionMesh::vertex_descriptor_t source =
	boost::source (edgeId_,
		       animatedMesh_->meshes ()[frameId_]->graph ());
      InteractionMesh::vertex_descriptor_t target =
	boost::target (edgeId_,
		       animatedMesh_->meshes ()[frameId_]->graph ());

      const Vertex& sourceVertex =
	animatedMesh_->meshes ()[frameId_]->graph ()[source];
      const Vertex& targetVertex =
	animatedMesh_->meshes ()[frameId_]->graph ()[target];
      const Edge& edge =
	animatedMesh_->meshes ()[frameId_]->graph ()[edgeId_];

      long unsigned int nVertices =
	boost::num_vertices (animatedMesh_->meshes ()[frameId_]->graph ());
      InteractionMeshShPtr_t newMesh =
	InteractionMesh::makeFromOptimizationVariables
	(x.segment (frameId_ * nVertices * 3, nVertices * 3));
      const Vertex& sourceVertexNew =
	newMesh->graph ()[source];
      const Vertex& targetVertexNew =
	newMesh->graph ()[target];

      result[1] =
	(targetVertex.position - sourceVertex.position).squaredNorm ();
      result[1] -= edge.scale *
	(targetVertexNew.position - sourceVertexNew.position).squaredNorm ();
    }

    void
    BoneLength::impl_gradient
    (gradient_t& gradient,
     const argument_t& x ,
     size_type i)
      const throw ()
    {
      roboptim::FiniteDifferenceGradient<>
	fdg (*this);
      fdg.gradient (gradient, x, i);
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
