#include <cassert>
#include "roboptim/retargeting/laplacian-coordinate.hh"

namespace roboptim
{
  namespace retargeting
  {
    LaplacianCoordinate::LaplacianCoordinate
    (InteractionMeshShPtr_t mesh,
     InteractionMesh::vertex_iterator_t vertex) throw ()
      : roboptim::Function (1, 3, "laplacian coordinate"),
	mesh_ (mesh),
	vertex_ (vertex)
    {
      assert (!!mesh_);
    }

    LaplacianCoordinate::~LaplacianCoordinate () throw ()
    {}

    void
    LaplacianCoordinate::impl_compute (result_t& result, const argument_t& x)
      const throw ()
    {
      assert (result.size () == 3);
      assert (x.size () - *vertex_ * 3 + 2 > 0);

      result[0] = x[*vertex_ * 3 + 0];
      result[1] = x[*vertex_ * 3 + 1];
      result[2] = x[*vertex_ * 3 + 2];

      InteractionMesh::index_map_t index =
	boost::get(boost::vertex_index, mesh_->graph ());
      std::pair<InteractionMesh::adjacency_iterator_t,
		InteractionMesh::adjacency_iterator_t> neighbors =
	boost::adjacent_vertices (vertex (1, mesh_->graph ()),
				  mesh_->graph ());
 
      for(; neighbors.first != neighbors.second; ++neighbors.first)
	{
	  roboptim::Function::size_type nodePosition =
	    index[*neighbors.first];

	  Eigen::Matrix<double, 3, 1> w;
	  w[0] = 1. /
	    (x[*vertex_ * 3 + 0] - x[nodePosition * 3 + 0]) *
	    (x[*vertex_ * 3 + 0] - x[nodePosition * 3 + 0]);
	  w[1] = 1. /
	    (x[*vertex_ * 3 + 1] - x[nodePosition * 3 + 1]) *
	    (x[*vertex_ * 3 + 1] - x[nodePosition * 3 + 1]);
	  w[2] = 1. /
	    (x[*vertex_ * 3 + 2] - x[nodePosition * 3 + 2]) *
	    (x[*vertex_ * 3 + 2] - x[nodePosition * 3 + 2]);

	  result[0] -= w[0] * x[nodePosition * 3 + 0];
	  result[1] -= w[1] * x[nodePosition * 3 + 1];
	  result[2] -= w[2] * x[nodePosition * 3 + 2];
	}
    }
   } // end of namespace retargeting.
} // end of namespace roboptim.
