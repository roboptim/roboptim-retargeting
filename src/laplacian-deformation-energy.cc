#include <roboptim/core/finite-difference-gradient.hh>

#include "roboptim/retargeting/laplacian-deformation-energy.hh"
#include "roboptim/retargeting/laplacian-coordinate.hh"

namespace roboptim
{
  namespace retargeting
  {
    LaplacianDeformationEnergy::LaplacianDeformationEnergy
    (AnimatedInteractionMeshShPtr_t animatedMesh) throw ()
      : roboptim::DifferentiableFunction
	(animatedMesh->optimizationVectorSize (),
	 1, ""),
	animatedMesh_ (animatedMesh),
	animatedMeshLocal_
	(AnimatedInteractionMesh::makeFromOptimizationVariables
	 (animatedMesh->state (), animatedMesh_))
    {}

    LaplacianDeformationEnergy::~LaplacianDeformationEnergy () throw ()
    {}

    void
    LaplacianDeformationEnergy::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      assert (result.size () == 1);

      AnimatedInteractionMesh::vertex_iterator_t vertexIt;
      AnimatedInteractionMesh::vertex_iterator_t vertexEnd;
      const unsigned& nVertices = animatedMesh_->numVertices ();

      animatedMeshLocal_->state () = x;
      animatedMeshLocal_->computeVertexWeights();

      for (unsigned frameId = 0;
	   frameId < animatedMesh_->numFrames (); ++frameId)
	{
	  unsigned vertexId = 0;
	  boost::tie (vertexIt, vertexEnd) = boost::vertices
	    (animatedMesh_->graph ());
	  for (; vertexIt != vertexEnd; ++vertexIt)
	    {
	      LaplacianCoordinate lcOrigin (animatedMesh_, *vertexIt, frameId);
	      LaplacianCoordinate lcNew (animatedMeshLocal_, *vertexIt, frameId);

	      result[0] += 
		(lcOrigin
		 (animatedMesh_->makeOptimizationVectorOneFrame (frameId))
		 - lcNew
		 (x.segment
		  (frameId * animatedMesh_->optimizationVectorSizeOneFrame (),
		   animatedMesh_->optimizationVectorSizeOneFrame ()))
		 ).squaredNorm ();
	      ++vertexId;
	    }
	}

      result[0] *= .5;
    }

    void
    LaplacianDeformationEnergy::impl_gradient
    (gradient_t& gradient,
     const argument_t& x,
     size_type i)
      const throw ()
    {
      roboptim::FiniteDifferenceGradient<>
	fdg (*this);
      fdg.gradient (gradient, x, i);
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.
