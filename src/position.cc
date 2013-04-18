#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/position.hh"

namespace roboptim
{
  namespace retargeting
  {
    Position::Position
    (AnimatedInteractionMeshShPtr_t animatedMesh,
     AnimatedInteractionMeshShPtr_t animatedMeshLocal,
     AnimatedInteractionMesh::vertex_descriptor_t vertexId,
     const Vertex::position_t& position) throw ()
      : roboptim::GenericLinearFunction<EigenMatrixSparse>
	(static_cast<size_type> (animatedMesh->optimizationVectorSize ()),
	 animatedMeshLocal_->numFrames () * 3,
	 "position"),
	animatedMesh_ (animatedMesh),
	animatedMeshLocal_ (animatedMeshLocal),
	vertexId_ (vertexId),
	position_ (position)
    {}

    Position::~Position () throw ()
    {}

    void
    Position::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      animatedMeshLocal_->state () = x;
      for (unsigned frameId = 0; frameId < animatedMeshLocal_->numFrames ();
	   ++frameId)
	result.segment (frameId * 3, 3) =
	  animatedMeshLocal_->graph ()[vertexId_].positions[frameId]
	  - position_;
    }

    void
    Position::impl_gradient
    (gradient_t& gradient,
     const argument_t&,
     size_type i)
      const throw ()
    {
      unsigned frameId = i / 3;
      unsigned offset = i % 3;
      gradient.insert
	(frameId * animatedMeshLocal_->numFrames () + offset) = 1.;
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
