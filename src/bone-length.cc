#include <boost/format.hpp>
#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/bone-length.hh"

namespace roboptim
{
  namespace retargeting
  {
    static std::string
    buildBoneLengthFunctionTitle
    (AnimatedInteractionMeshShPtr_t animatedMesh,
     AnimatedInteractionMesh::edge_descriptor_t edgeId)
    {
      std::string vertexSource =
	animatedMesh->graph ()
	[boost::source
	 (edgeId,
	  animatedMesh->graph ())].label;
      std::string vertexDest =
	animatedMesh->graph ()
	[boost::target
	 (edgeId,
	  animatedMesh->graph ())].label;
      const double& scale = animatedMesh->graph ()[edgeId].scale;

      return (boost::format ("edge id = [%1%, %2%], scale = %3%")
	      % vertexSource
	      % vertexDest
	      % scale).str ();
    }

    BoneLength::BoneLength
    (AnimatedInteractionMeshShPtr_t animatedMesh,
     AnimatedInteractionMeshShPtr_t animatedMeshLocal,
     AnimatedInteractionMesh::edge_descriptor_t edgeId) throw ()
      : roboptim::LinearFunction
	(animatedMesh->optimizationVectorSize (),
	 animatedMesh->numFrames (),
	 (boost::format ("bone length (%1%)")
	  % buildBoneLengthFunctionTitle (animatedMesh, edgeId)).str ()),
	animatedMesh_ (animatedMesh),
	animatedMeshLocal_ (animatedMeshLocal),
	edgeId_ (edgeId),
	source_ (boost::source (edgeId_, animatedMesh_->graph ())),
	target_ (boost::target (edgeId_, animatedMesh_->graph ())),
	scale_ (animatedMesh->graph ()[edgeId].scale),
	goalLength_ ()
    {
      const Vertex& sourceVertex = animatedMesh_->graph ()[source_];
      const Vertex& targetVertex = animatedMesh_->graph ()[target_];

      goalLength_ =
	scale_ *
	(targetVertex.positions[0]
	 - sourceVertex.positions[0]).squaredNorm ();
    }

    BoneLength::~BoneLength () throw ()
    {}

    void
    BoneLength::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      animatedMeshLocal_->state () = x;
      animatedMeshLocal_->computeVertexWeights();

      const Vertex& sourceVertexNew =
	animatedMeshLocal_->graph ()[source_];
      const Vertex& targetVertexNew =
	animatedMeshLocal_->graph ()[target_];

      for (unsigned i = 0; i < animatedMesh_->numFrames (); ++i)
	result[i] =
	    (targetVertexNew.positions[i]
	     - sourceVertexNew.positions[i]).squaredNorm ()
	  - goalLength_;
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
