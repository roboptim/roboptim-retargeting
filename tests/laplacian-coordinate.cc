#include <boost/make_shared.hpp>
#include <roboptim/retargeting/laplacian-coordinate.hh>
#include <roboptim/retargeting/interaction-mesh.hh>

#define BOOST_TEST_MODULE angle

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (simple)
{
  roboptim::retargeting::InteractionMeshShPtr_t mesh =
    boost::make_shared<roboptim::retargeting::InteractionMesh> ();

  // Add two vertices.
  roboptim::retargeting::InteractionMesh::vertex_descriptor_t v0 =
    boost::add_vertex (mesh->graph ());
  roboptim::retargeting::InteractionMesh::vertex_descriptor_t v1 =
    boost::add_vertex (mesh->graph ());

  // Link them using an edge.
  boost::add_edge (v0, v1, mesh->graph ());

  roboptim::Function::argument_t x (2);
  roboptim::retargeting::LaplacianCoordinate lc
    (mesh,
     std::find (boost::vertices (mesh->graph ()).first,
		boost::vertices (mesh->graph ()).second,
		v0));
}
