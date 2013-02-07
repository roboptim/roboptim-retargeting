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

    template <typename U, typename V>
    void
    computeGradient
    (Eigen::MatrixBase<U>& gradient,
     const Eigen::MatrixBase<V>& arg,
     Function::size_type frameId,
     AnimatedInteractionMeshShPtr_t animatedMesh_,
     AnimatedInteractionMesh::vertex_descriptor_t source_,
     AnimatedInteractionMesh::vertex_descriptor_t target_)
    {
      const unsigned& nVertices = animatedMesh_->numVertices ();
      const double& sourceX =
	arg[frameId * 3 * nVertices + source_ * 3 + 0];
      const double& sourceY =
	arg[frameId * 3 * nVertices + source_ * 3 + 1];
      const double& sourceZ =
	arg[frameId * 3 * nVertices + source_ * 3 + 2];
      const double& targetX =
	arg[frameId * 3 * nVertices + target_ * 3 + 0];
      const double& targetY =
	arg[frameId * 3 * nVertices + target_ * 3 + 1];
      const double& targetZ =
	arg[frameId * 3 * nVertices + target_ * 3 + 2];

      // derivative w.r.t x position
      gradient(frameId * 3 * nVertices + source_ * 3 + 0, 0) = 2 * sourceX;
      // derivative w.r.t y position
      gradient(frameId * 3 * nVertices + source_ * 3 + 1, 0) = 2 * sourceY;
      // derivative w.r.t z position
      gradient(frameId * 3 * nVertices + source_ * 3 + 2, 0) = 2 * sourceZ;

      // derivative w.r.t x position
      gradient(frameId * 3 * nVertices + target_ * 3 + 0, 0) = -2 * targetX;
      // derivative w.r.t y position
      gradient(frameId * 3 * nVertices + target_ * 3 + 1, 0) = -2 * targetY;
      // derivative w.r.t z position
      gradient(frameId * 3 * nVertices + target_ * 3 + 2, 0) = -2 * targetZ;
    }

    void
    BoneLength::impl_gradient
    (gradient_t& gradient,
     const argument_t& arg,
     size_type frameId)
      const throw ()
    {
      computeGradient (gradient, arg, frameId, animatedMesh_, source_, target_);
    }

    void
    BoneLength::impl_jacobian
    (jacobian_t& jacobian,
     const argument_t& arg)
      const throw ()
    {
      jacobian.setZero ();
      for (unsigned frameId = 0 ;
	   frameId < animatedMesh_->numFrames (); ++frameId)
	{
	  Eigen::Block<Eigen::MatrixXd> gradient =
	    jacobian.block (frameId, 0, 1, jacobian.cols ());
	  computeGradient
	    (gradient, arg, frameId, animatedMesh_, source_, target_);
	}
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
