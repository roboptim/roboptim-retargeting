#include <boost/make_shared.hpp>
#include <roboptim/retargeting/laplacian-deformation-energy.hh>
#include <roboptim/retargeting/interaction-mesh.hh>

#define BOOST_TEST_MODULE laplacian_deformation_energy

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

  mesh->graph ()[v0].id = 0;
  mesh->graph ()[v0].position[0] = 0.;
  mesh->graph ()[v0].position[1] = 0.;
  mesh->graph ()[v0].position[2] = 0.;

  roboptim::retargeting::InteractionMesh::vertex_descriptor_t v1 =
    boost::add_vertex (mesh->graph ());

  mesh->graph ()[v1].id = 1;
  mesh->graph ()[v1].position[0] = 1.;
  mesh->graph ()[v1].position[1] = 1.;
  mesh->graph ()[v1].position[2] = 1.;

  // Link them using an edge.
  boost::add_edge (v0, v1, mesh->graph ());

  mesh->computeVertexWeights ();

  roboptim::Function::argument_t x (2 * 3);
  for (unsigned i = 0; i < x.size (); ++i)
    x[i] = 0.;
  roboptim::retargeting::LaplacianDeformationEnergy lde (mesh);

  std::cout << lde.inputSize () << std::endl;
  std::cout << lde (x) << std::endl;

}
