#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/acceleration-energy.hh"

namespace roboptim
{
  namespace retargeting
  {
    AccelerationEnergy::AccelerationEnergy
    (AnimatedInteractionMeshShPtr_t animatedMesh) throw ()
      : roboptim::DifferentiableFunction
	(animatedMesh->optimizationVectorSize (),
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

      // We exclude first and last frame for which the contribution of
      // energy is zero.
      for (unsigned frame = 1;
	   frame < animatedMesh_->meshes ().size () - 1; ++frame)
	{
	  InteractionMeshShPtr_t mesh = animatedMesh_->meshes ()[frame];
	  InteractionMesh::vertex_iterator_t vertexIt;
	  InteractionMesh::vertex_iterator_t vertexEnd;

	  long unsigned nVertices = boost::num_vertices (mesh->graph ());
	  boost::tie (vertexIt, vertexEnd) = boost::vertices (mesh->graph ());
	  for (long unsigned vertex = 0; vertex < nVertices; ++vertex)
	    {
	      long unsigned variableId = frame * nVertices + vertex;
	      long unsigned variableIdPreviousFrame =
		(frame - 1) * nVertices + vertex;
	      long unsigned variableIdNextFrame =
		(frame + 1) * nVertices + vertex;

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
    }

    void
    AccelerationEnergy::impl_gradient
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
