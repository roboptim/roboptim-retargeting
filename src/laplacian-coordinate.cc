#include <cassert>
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
    log4cxx::LoggerPtr LaplacianCoordinate::logger
    (log4cxx::Logger::getLogger("roboptim.retargeting.LaplacianCoordinate"));

    LaplacianCoordinate::LaplacianCoordinate
    (AnimatedInteractionMeshShPtr_t mesh,
     AnimatedInteractionMesh::vertex_descriptor_t vertex,
     unsigned frameId) throw ()
      : roboptim::Function
	(mesh->optimizationVectorSizeOneFrame (), 3,
	 "laplacian coordinate"),
	mesh_ (mesh),
	vertex_ (vertex),
	frameId_ (frameId)
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
	(logger, "Vertex id: " << vertex_);

      result = x.segment (vertex_ * 3, 3);

      LOG4CXX_TRACE (logger,
		     "Euclidian position: "
		     << result[0] << " "
		     << result[1] << " "
		     << result[2]);

      AnimatedInteractionMesh::out_edge_iterator_t edgeIt;
      AnimatedInteractionMesh::out_edge_iterator_t edgeEnd;
      boost::tie(edgeIt, edgeEnd) =
	boost::out_edges (vertex_, mesh_->graph ());

      for(; edgeIt != edgeEnd; ++edgeIt)
	{
	  const Edge& edge = mesh_->graph()[*edgeIt];

	  AnimatedInteractionMesh::vertex_descriptor_t source =
	    boost::source (*edgeIt, mesh_->graph ());
	  AnimatedInteractionMesh::vertex_descriptor_t target =
	    boost::target (*edgeIt, mesh_->graph ());
	  AnimatedInteractionMesh::vertex_descriptor_t
	    neighborDesc = (vertex_ == source) ? target : source;
	  const Vertex& neighbor = mesh_->graph ()[neighborDesc];

	  LOG4CXX_TRACE
	    (logger,
	     "--- edge ---\n"
	     << "Edge weight: " << edge.weight[frameId_] << "\n"
	     << "Vertex id: " << neighborDesc << "\n"
	     << "Euclidian position: "
	     << neighbor.positions[frameId_][0] << " "
	     << neighbor.positions[frameId_][1] << " "
	     << neighbor.positions[frameId_][2]);

	  result -= neighbor.positions[frameId_] * edge.weight[frameId_];
	}
    }
   } // end of namespace retargeting.
} // end of namespace roboptim.
