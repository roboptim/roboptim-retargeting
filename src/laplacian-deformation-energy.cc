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

      for (unsigned i = 0; i < animatedMesh_->meshes ().size (); ++i)
	{
	  InteractionMeshShPtr_t mesh = animatedMesh_->meshes ()[i];
	  InteractionMesh::vertex_iterator_t vertexIt;
	  InteractionMesh::vertex_iterator_t vertexEnd;
	  
	  unsigned i = 0;
	  long unsigned int nVertices = boost::num_vertices (mesh->graph ());
	  boost::tie (vertexIt, vertexEnd) = boost::vertices (mesh->graph ());
	  for (; vertexIt != vertexEnd; ++vertexIt)
	    {
	      LaplacianCoordinate lcOrigin (mesh, *vertexIt);
	      
	      InteractionMeshShPtr_t newMesh =
		InteractionMesh::makeFromOptimizationVariables
		(x.segment (i * nVertices * 3, nVertices * 3));
	      LaplacianCoordinate lcNew (newMesh, *vertexIt);

	      result[0] += (lcOrigin
			    (mesh->optimizationVector ())
			    - lcNew 
			    (x.segment (i * nVertices * 3, nVertices * 3))
			    ).squaredNorm ();
	      ++i;
	    }
	}

      result[0] *= .5;
    }

    void
    LaplacianDeformationEnergy::impl_gradient
    (gradient_t&,
     const argument_t&,
     size_type)
      const throw ()
    {}

  } // end of namespace retargeting.
} // end of namespace roboptim.
