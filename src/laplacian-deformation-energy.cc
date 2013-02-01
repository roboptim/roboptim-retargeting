#include <boost/make_shared.hpp>
#include <roboptim/core/finite-difference-gradient.hh>

#include "roboptim/retargeting/laplacian-deformation-energy.hh"
#include "roboptim/retargeting/laplacian-coordinate.hh"

// Remove trace logging in release.
#ifdef NDEBUG
# undef LOG4CXX_TRACE
# define LOG4CXX_TRACE(logger, msg)
#endif //!NDEBUG

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
	 (animatedMesh->state (), animatedMesh_)),
	laplacianCoordinates_ (),
	laplacianCoordinatesLocal_ (),
	buffer_ (3)
    {
      AnimatedInteractionMesh::vertex_iterator_t vertexIt;
      AnimatedInteractionMesh::vertex_iterator_t vertexEnd;

      for (unsigned frameId = 0; frameId < animatedMesh_->numFrames ();
	   ++frameId)
	{
	  boost::tie (vertexIt, vertexEnd) = boost::vertices
	    (animatedMesh_->graph ());
	  for (; vertexIt != vertexEnd; ++vertexIt)
	    
	    {
	      // Precompute original mesh laplacian coordinates.
	      laplacianCoordinates_.push_back
		(LaplacianCoordinate
		 (animatedMesh_, *vertexIt, frameId)
		 (animatedMesh_->state ().segment
		  (frameId *
		   animatedMesh_->optimizationVectorSizeOneFrame (),
		   animatedMesh_->optimizationVectorSizeOneFrame ())
		  ));

	      laplacianCoordinatesLocal_.push_back
		(boost::make_shared<LaplacianCoordinate>
		 (animatedMeshLocal_, *vertexIt, frameId));
	    }
	}
    }

    LaplacianDeformationEnergy::~LaplacianDeformationEnergy () throw ()
    {}

    void
    LaplacianDeformationEnergy::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

      assert (result.size () == 1);

      AnimatedInteractionMesh::vertex_iterator_t vertexIt;
      AnimatedInteractionMesh::vertex_iterator_t vertexEnd;
      const unsigned& nVertices = animatedMesh_->numVertices ();

      animatedMeshLocal_->state () = x;
      animatedMeshLocal_->computeVertexWeights();

      result[0] = 0.;

      for (unsigned frameId = 0;
	   frameId < animatedMesh_->numFrames (); ++frameId)
	{
	  unsigned vertexId = 0;
	  boost::tie (vertexIt, vertexEnd) = boost::vertices
	    (animatedMesh_->graph ());
	  for (; vertexIt != vertexEnd; ++vertexIt, ++vertexId)
	    {
	      (*laplacianCoordinatesLocal_[frameId * nVertices + vertexId])
		(buffer_,
		 x.segment
		 (frameId * animatedMesh_->optimizationVectorSizeOneFrame (),
		  animatedMesh_->optimizationVectorSizeOneFrame ()));
		
	      result[0] += 
		(laplacianCoordinates_[frameId * nVertices + vertexId]
		 - buffer_).squaredNorm ();
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
