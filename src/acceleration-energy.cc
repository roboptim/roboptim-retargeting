#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/acceleration-energy.hh"

namespace roboptim
{
  namespace retargeting
  {
    AccelerationEnergy::AccelerationEnergy
    (AnimatedInteractionMeshShPtr_t animatedMesh) throw ()
      : roboptim::GenericDifferentiableFunction<EigenMatrixSparse>
	(static_cast<size_type> (animatedMesh->optimizationVectorSize ()),
	 1, ""),
	animatedMesh_ (animatedMesh)
    {}

    AccelerationEnergy::~AccelerationEnergy () throw ()
    {}

    void
    AccelerationEnergy::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      result.resize (1);

      // Set result to zero and add the energy contribution of each
      // vertex of each frame separately.
      result[0] = 0.;

      const unsigned& numFrames = animatedMesh_->numFrames ();
      const unsigned& numVertices = animatedMesh_->numVertices ();

      // We exclude first and last frame for which the contribution of
      // energy is zero.
      for (unsigned frame = 1;
	   frame < numFrames - 1; ++frame)
	{
	  AnimatedInteractionMesh::vertex_iterator_t vertexIt;
	  AnimatedInteractionMesh::vertex_iterator_t vertexEnd;

	  boost::tie (vertexIt, vertexEnd) =
	    boost::vertices (animatedMesh_->graph ());
	  for (long unsigned vertex = 0; vertex < numVertices; ++vertex)
	    {
	      long unsigned variableId = frame * numVertices + vertex;
	      long unsigned variableIdPreviousFrame =
		(frame - 1) * numVertices + vertex;
	      long unsigned variableIdNextFrame =
		(frame + 1) * numVertices + vertex;

	      // Compute velocity from position by finite differentiation.
	      double h = 1. / animatedMesh_->framerate ();
	      result[0] +=
		(x[variableIdNextFrame] 
		 - 2 * x[variableId]
		 + x[variableIdPreviousFrame])
		 / (h * h);
	    }
	}

      result[0] = .5 * result[0] * result[0];

      result[0] *= 0.2; // weight on acceleration, should not be here.
    }

    void
    AccelerationEnergy::impl_gradient
    (gradient_t& gradient,
     const argument_t& x,
     size_type i)
      const throw ()
    {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

      roboptim::GenericFiniteDifferenceGradient<
	EigenMatrixSparse,
	finiteDifferenceGradientPolicies::Simple<EigenMatrixSparse> >
	fdg (*this);
      fdg.gradient (gradient, x, i);
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
