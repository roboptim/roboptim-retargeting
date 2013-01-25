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
	animatedMesh_ (animatedMesh)
    {}

    LaplacianDeformationEnergy::~LaplacianDeformationEnergy () throw ()
    {}

    void
    LaplacianDeformationEnergy::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      result.resize (1);

      for (unsigned frameId = 0;
	   frameId < animatedMesh_->meshes ().size (); ++frameId)
	{
	  InteractionMeshShPtr_t mesh = animatedMesh_->meshes ()[frameId];
	  InteractionMesh::vertex_iterator_t vertexIt;
	  InteractionMesh::vertex_iterator_t vertexEnd;

	  unsigned vertexId = 0;
	  long unsigned int nVertices = boost::num_vertices (mesh->graph ());
	  boost::tie (vertexIt, vertexEnd) = boost::vertices (mesh->graph ());
	  for (; vertexIt != vertexEnd; ++vertexIt)
	    {
	      LaplacianCoordinate lcOrigin (mesh, *vertexIt);

	      InteractionMeshShPtr_t newMesh =
		InteractionMesh::makeFromOptimizationVariables
		(x.segment
		 (vertexId * nVertices * 3, nVertices * 3));
	      LaplacianCoordinate lcNew (newMesh, *vertexIt);

	      result[0] += (lcOrigin
			    (mesh->optimizationVector ())
			    - lcNew
			    (x.segment
			     (vertexId * nVertices * 3, nVertices * 3))
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
