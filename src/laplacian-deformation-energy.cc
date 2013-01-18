#include "roboptim/retargeting/laplacian-deformation-energy.hh"
#include "roboptim/retargeting/laplacian-coordinate.hh"

namespace roboptim
{
  namespace retargeting
  {
    LaplacianDeformationEnergy::LaplacianDeformationEnergy
    (InteractionMeshShPtr_t mesh) throw ()
      : roboptim::Function
	(boost::num_vertices (mesh->graph ()) * 3 * 1,
	 1, ""),
	mesh_ (mesh)
    {}

    LaplacianDeformationEnergy::~LaplacianDeformationEnergy () throw ()
    {}

    void
    LaplacianDeformationEnergy::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      result.resize (1);

      InteractionMesh::vertex_iterator_t vertexIt;
      InteractionMesh::vertex_iterator_t vertexEnd;

      boost::tie (vertexIt, vertexEnd) = boost::vertices (mesh_->graph ());
      for (; vertexIt != vertexEnd; ++vertexIt)
	{
	  LaplacianCoordinate lcOrigin (mesh_, *vertexIt);

	  //FIXME: mesh should be different.
	  LaplacianCoordinate lcNew (mesh_, *vertexIt);
	  result[0] += (lcOrigin (x) - lcNew (x)).squaredNorm ();
	}

      result[0] *= .5;
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
