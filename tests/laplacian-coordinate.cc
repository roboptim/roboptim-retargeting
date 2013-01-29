#include <boost/make_shared.hpp>
#include <log4cxx/basicconfigurator.h>
#include <roboptim/retargeting/laplacian-coordinate.hh>
#include <roboptim/retargeting/animated-interaction-mesh.hh>

#define BOOST_TEST_MODULE laplacian_coordinate

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include "tests-config.h"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (simple)
{
  configureLog4cxx ();

  log4cxx::LoggerPtr logger
    (log4cxx::Logger::getLogger
     ("roboptim.retargeting.tests.laplacian-coordinate"));

  roboptim::retargeting::AnimatedInteractionMeshShPtr_t mesh =
    boost::make_shared<roboptim::retargeting::AnimatedInteractionMesh> ();

  // Add two vertices.
  roboptim::retargeting::AnimatedInteractionMesh::vertex_descriptor_t v0 =
    boost::add_vertex (mesh->graph ());

  mesh->graph ()[v0].positions.resize (1);
  mesh->graph ()[v0].positions[0][0] = 0.;
  mesh->graph ()[v0].positions[0][1] = 0.;
  mesh->graph ()[v0].positions[0][2] = 0.;

  roboptim::retargeting::AnimatedInteractionMesh::vertex_descriptor_t v1 =
    boost::add_vertex (mesh->graph ());

  mesh->graph ()[v1].positions.resize (1);
  mesh->graph ()[v1].positions[0][0] = 1.;
  mesh->graph ()[v1].positions[0][1] = 1.;
  mesh->graph ()[v1].positions[0][2] = 1.;

  // Link them using an edge.
  boost::add_edge (v0, v1, mesh->graph ());

  mesh->computeVertexWeights ();

  roboptim::Function::argument_t x (2 * 3);
  for (unsigned i = 0; i < x.size (); ++i)
    x[i] = 0.;
  roboptim::retargeting::LaplacianCoordinate lc (mesh, v0, 0);

  LOG4CXX_INFO (logger, lc.inputSize ());
  LOG4CXX_INFO (logger, lc (x));

}
