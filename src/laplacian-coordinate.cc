#include <cassert>
#include <roboptim/core/finite-difference-gradient.hh>
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
      : roboptim::LinearFunction
	(mesh->optimizationVectorSizeOneFrame (), 3,
	 "laplacian coordinate"),
	mesh_ (mesh),
	vertex_ (vertex),
	frameId_ (frameId)
    {
      assert (!!mesh_);
      assert (frameId < mesh->numFrames ());
    }

    LaplacianCoordinate::~LaplacianCoordinate () throw ()
    {}

    void
    LaplacianCoordinate::impl_compute (result_t& result, const argument_t& x)
      const throw ()
    {
      assert (result.size () == 3);
      assert (x.size () - vertex_ * 3 + 2 > 0);

      LOG4CXX_TRACE
	(logger, "Vertex id: " << vertex_);

      result = x.segment (vertex_ * 3, 3);

      LOG4CXX_TRACE (logger,
		     "Euclidian position: "
		     << result[0] << " "
		     << result[1] << " "
		     << result[2]);

      AnimatedInteractionMesh::interaction_mesh_out_edge_iterator_t edgeIt;
      AnimatedInteractionMesh::interaction_mesh_out_edge_iterator_t edgeEnd;

      assert (mesh_->interactionMeshes ().size () > frameId_);
      boost::tie(edgeIt, edgeEnd) =
	boost::out_edges (vertex_, mesh_->interactionMeshes ()[frameId_]);

      for(; edgeIt != edgeEnd; ++edgeIt)
	{
	  const InteractionMeshEdge& edge =
	    mesh_->interactionMeshes ()[frameId_][*edgeIt];

	  AnimatedInteractionMesh::interaction_mesh_vertex_descriptor_t source =
	    boost::source (*edgeIt, mesh_->interactionMeshes ()[frameId_]);
	  AnimatedInteractionMesh::interaction_mesh_vertex_descriptor_t target =
	    boost::target (*edgeIt, mesh_->interactionMeshes ()[frameId_]);
	  AnimatedInteractionMesh::interaction_mesh_vertex_descriptor_t
	    neighborDesc = (vertex_ == source) ? target : source;
	  const InteractionMeshVertex& interactiveMeshNeighbor =
	    mesh_->interactionMeshes ()[frameId_][neighborDesc];
	  const Vertex& neighbor = mesh_->graph ()[neighborDesc];

	  LOG4CXX_TRACE
	    (logger,
	     "--- edge ---\n"
	     << "Edge weight: " << edge.weight << "\n"
	     << "Vertex id: " << neighborDesc << "\n"
	     << "Euclidian position: "
	     << neighbor.positions[frameId_][0] << " "
	     << neighbor.positions[frameId_][1] << " "
	     << neighbor.positions[frameId_][2]);

	  result -= neighbor.positions[frameId_] * edge.weight;
	}
    }

    void
    LaplacianCoordinate::impl_gradient (gradient_t& gradient,
					const argument_t& argument,
					size_type functionId)
      const throw ()
    {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

      roboptim::GenericFiniteDifferenceGradient<
	EigenMatrixDense,
	finiteDifferenceGradientPolicies::Simple<EigenMatrixDense> >
	fdg (*this);

      fdg.gradient (gradient, argument, functionId);
    }

   } // end of namespace retargeting.
} // end of namespace roboptim.
