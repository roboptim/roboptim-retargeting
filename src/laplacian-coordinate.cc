#include <cassert>
#include "roboptim/retargeting/laplacian-coordinate.hh"

namespace roboptim
{
  namespace retargeting
  {
    log4cxx::LoggerPtr LaplacianCoordinate::logger
    (log4cxx::Logger::getLogger("roboptim.retargeting.LaplacianCoordinate"));

    LaplacianCoordinate::LaplacianCoordinate
    (InteractionMeshShPtr_t mesh,
     InteractionMesh::vertex_descriptor_t vertex) throw ()
      : roboptim::Function
	(mesh->optimizationVectorSize (), 3,
	 "laplacian coordinate"),
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
      assert (x.size () - vertex_ * 3 + 2 > 0);

      const Vertex& vertex = mesh_->graph ()[vertex_];

      LOG4CXX_TRACE
	(logger, "Vertex id: " << vertex.id);

      result[0] = x[vertex.id * 3 + 0];
      result[1] = x[vertex.id * 3 + 1];
      result[2] = x[vertex.id * 3 + 2];

      LOG4CXX_TRACE (logger,
		     "Euclidian position: "
		     << result[0] << " "
		     << result[1] << " "
		     << result[2]);

      InteractionMesh::out_edge_iterator_t edgeIt;
      InteractionMesh::out_edge_iterator_t edgeEnd;
      boost::tie(edgeIt, edgeEnd) =
	boost::out_edges (vertex_, mesh_->graph ());

      for(; edgeIt != edgeEnd; ++edgeIt)
	{
	  const Edge& edge = mesh_->graph()[*edgeIt];

	  InteractionMesh::vertex_descriptor_t source =
	    boost::source (*edgeIt, mesh_->graph ());
	  InteractionMesh::vertex_descriptor_t target =
	    boost::target (*edgeIt, mesh_->graph ());
	  const Vertex& neighbor = (vertex_ == source)
	    ? mesh_->graph ()[target]
	    : mesh_->graph ()[source];

	  LOG4CXX_TRACE
	    (logger,
	     "--- edge ---\n"
	     << "Edge weight: " << edge.weight << "\n"
	     << "Vertex id: " << neighbor.id << "\n"
	     << "Euclidian position: "
	     << neighbor.position[0] << " "
	     << neighbor.position[1] << " "
	     << neighbor.position[2] << "\n"
	     "w: " << edge.weight);

	  result -= neighbor.position * edge.weight;
	}
    }
   } // end of namespace retargeting.
} // end of namespace roboptim.
