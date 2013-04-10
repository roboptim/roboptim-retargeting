#include <boost/make_shared.hpp>
#include <log4cxx/basicconfigurator.h>

//FIXME: kill this before it lays eggs.
#define private public
#include <roboptim/retargeting/laplacian-coordinate.hh>
#include <roboptim/retargeting/animated-interaction-mesh.hh>
#undef private

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

  mesh->framerate_ = 30;
  mesh->numFrames_ = 1;
  mesh->numVertices_ = 2;
  mesh->state ().resize (3 * 2);
  mesh->state ().setZero ();
  for (unsigned i = 3; i < 6; ++i)
    mesh->state ()[i] = 1.;

  // Add two vertices.
  roboptim::retargeting::AnimatedInteractionMesh::vertex_descriptor_t v0 =
    boost::add_vertex (mesh->graph ());

  mesh->graph ()[v0].positions.push_back
    (roboptim::retargeting::Vertex::position_t (mesh->state (), 0, 3));
    
  roboptim::retargeting::AnimatedInteractionMesh::vertex_descriptor_t v1 =
    boost::add_vertex (mesh->graph ());
    
  mesh->graph ()[v1].positions.push_back
    (roboptim::retargeting::Vertex::position_t (mesh->state (), 3, 3));
    
  // Link them using an edge.
  boost::add_edge (v0, v1, mesh->graph ());

  // Add two vertices (interaction mesh).
  mesh->interactionMeshes ().resize (1);
  {
    roboptim::retargeting::AnimatedInteractionMesh::vertex_descriptor_t v0 =
      boost::add_vertex (mesh->interactionMeshes ()[0]);    
    roboptim::retargeting::AnimatedInteractionMesh::vertex_descriptor_t v1 =
      boost::add_vertex (mesh->interactionMeshes ()[0]);
    // Link them using an edge.
    boost::add_edge (v0, v1, mesh->interactionMeshes ()[0]);
  }
    
  mesh->recomputeCachedData ();

  roboptim::Function::argument_t x (2 * 3);
  for (unsigned i = 0; i < x.size (); ++i)
    x[i] = 0.;
  roboptim::retargeting::LaplacianCoordinate lc (mesh, v0, 0);

  LOG4CXX_INFO (logger, lc.inputSize ());
  LOG4CXX_INFO (logger, lc (x));

}
