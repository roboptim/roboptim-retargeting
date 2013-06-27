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
      : roboptim::GenericLinearFunction<EigenMatrixSparse>
	(static_cast<size_type> (animatedMesh->optimizationVectorSize ()),
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
	.5 * scale_ *
	(targetVertex.positions[0]
	 - sourceVertex.positions[0]).squaredNorm ();
    }

    BoneLength::~BoneLength () throw ()
    {}

void
BoneLength::setBoneLengthMatrixAndVectorOfFrame(matrix_t& Hi, double alpha)
{
    int globalBoneIndex = 0;
    
    for(size_t motionIndex=0; motionIndex < characterInfos.size(); ++motionIndex){

        CharacterInfo& chara = characterInfos[motionIndex];
        if(chara.org){
            const cnoid::MarkerMotion::Frame& Vi0_frame = Vi0_frames[motionIndex];
            const int numBones = chara.bones.size();
            for(int j=0; j < numBones; ++j){
                const Bone& bone = chara.bones[j];
                cnoid::MarkerMotionPtr motion = mesh->motion(motionIndex);

                // Set desired bone length in this morph step
                const cnoid::Vector3& pe1_0 = Vi0_frame[bone.localMarkerIndex1];
                const cnoid::Vector3& pe2_0 = Vi0_frame[bone.localMarkerIndex2];
                double lgoal = (pe2_0 - pe1_0).norm(); // set original length first
                if(bone.goalLength){
                    lgoal = alpha * (*bone.goalLength) + (1.0 - alpha) * lgoal;
                }
                
                const cnoid::MarkerMotion::Frame& Vi_frame = Vi_frames[motionIndex];
                const cnoid::Vector3& pe1 = Vi_frame[bone.localMarkerIndex1];
                const cnoid::Vector3& pe2 = Vi_frame[bone.localMarkerIndex2];

                const double lcurrent = (pe1 - pe2).norm();

                double a;
                if(fabs(lcurrent) < 1.0e-6){
                    a = 0.0; // to make the following coefficient zero
                } else {
                    a = lgoal / lcurrent;
                }

                hi(globalBoneIndex) = (lgoal - lcurrent);

                cnoid::Vector3 dl;
                if(fabs(lcurrent) < 1.0e-6){
                    dl.setZero();
                } else {
                    dl = (1.0 / lcurrent) * (pe1 - pe2);
                }

                Hi.startVec(globalBoneIndex);
                
                int offset = bone.activeVertexIndex1 * 3;
                for(int j=0; j < 3; ++j){
                    Hi.insertBack(globalBoneIndex, offset + j) = dl[j];
                    hi(globalBoneIndex) += dl[j] * pe1[j];
                }
                offset = bone.activeVertexIndex2 * 3;
                for(int j=0; j < 3; ++j){
                    Hi.insertBack(globalBoneIndex, offset + j) = -dl[j];
                    hi(globalBoneIndex) -= dl[j] * pe2[j];
                }
                
                ++globalBoneIndex;
            }
        }
    }
}


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
	  (.5 *
	   (targetVertexNew.positions[i]
	    - sourceVertexNew.positions[i]).squaredNorm ())
	  - goalLength_;
    }

    template <typename U, typename V>
    void
    computeGradient
    (U& gradient,
     const V&,
     Function::size_type frameId,
     AnimatedInteractionMeshShPtr_t animatedMeshLocal,
     AnimatedInteractionMesh::vertex_descriptor_t source_,
     AnimatedInteractionMesh::vertex_descriptor_t target_)
    {
      const unsigned oneFrameOffset = (unsigned)
	animatedMeshLocal->optimizationVectorSizeOneFrame ();
      const unsigned sourceOffset = (unsigned) source_;
      const unsigned targetOffset = (unsigned) target_;

      const Vertex& sourceVertexNew =
	animatedMeshLocal->graph ()[source_];
      const Vertex& targetVertexNew =
	animatedMeshLocal->graph ()[target_];

      const double& sourceX =
	sourceVertexNew.positions[frameId][0];
      const double& sourceY =
	sourceVertexNew.positions[frameId][1];
      const double& sourceZ =
	sourceVertexNew.positions[frameId][2];

      const double& targetX =
	targetVertexNew.positions[frameId][0];
      const double& targetY =
	targetVertexNew.positions[frameId][1];
      const double& targetZ =
	targetVertexNew.positions[frameId][2];

      // derivative w.r.t x position
      gradient.insert (frameId * oneFrameOffset + sourceOffset * 3 + 0) =
	(sourceX - targetX);
      // derivative w.r.t y position
      gradient.insert (frameId * oneFrameOffset + sourceOffset * 3 + 1) =
	(sourceY - targetY);
      // derivative w.r.t z position
      gradient.insert (frameId * oneFrameOffset + sourceOffset * 3 + 2) =
	(sourceZ - targetZ);

      // derivative w.r.t x position
      gradient.insert (frameId * oneFrameOffset + targetOffset * 3 + 0) =
	(targetX - sourceX);
      // derivative w.r.t y position
      gradient.insert (frameId * oneFrameOffset + targetOffset * 3 + 1) =
	(targetY - sourceY);
      // derivative w.r.t z position
      gradient.insert (frameId * oneFrameOffset + targetOffset * 3 + 2) =
	(targetZ - sourceZ);
    }

    void
    BoneLength::impl_gradient
    (gradient_t& gradient,
     const argument_t& arg,
     size_type frameId)
      const throw ()
    {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

      animatedMeshLocal_->state () = arg;
      animatedMeshLocal_->computeVertexWeights();

      computeGradient (gradient, arg, frameId, animatedMeshLocal_,
      		       source_, target_);
    }

    void
    BoneLength::impl_jacobian
    (jacobian_t& jacobian,
     const argument_t& arg)
      const throw ()
    {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

      animatedMeshLocal_->state () = arg;
      animatedMeshLocal_->computeVertexWeights();

      jacobian.setZero ();
      for (unsigned frameId = 0 ;
	   frameId < animatedMesh_->numFrames (); ++frameId)
	{
	  gradient_t g (inputSize ());
	  computeGradient
	    (g, arg, frameId, animatedMeshLocal_, source_, target_);
	  jacobian.middleRows (frameId, 1) = g;
	}
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
