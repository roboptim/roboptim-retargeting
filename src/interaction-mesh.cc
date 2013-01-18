#include <boost/make_shared.hpp>

#include <fstream>
#include <yaml-cpp/iterator.h>
#include <yaml-cpp/yaml.h>

#include "roboptim/retargeting/interaction-mesh.hh"

namespace roboptim
{
  namespace retargeting
  {
    InteractionMesh::InteractionMesh ()
    {}

    InteractionMesh::~InteractionMesh ()
    {}

    void
    InteractionMesh::computeVertexWeights ()
    {
      edge_iterator_t edgeIt;
      edge_iterator_t edgeEnd;

      double weightSum = 0.;

      boost::tie (edgeIt, edgeEnd) = boost::edges (graph ());
      for (; edgeIt != edgeEnd; ++edgeIt)
	{
	  Edge& edge = graph ()[*edgeIt];
	  const Vertex& source = graph ()[boost::source (*edgeIt, graph ())];
	  const Vertex& target = graph ()[boost::target (*edgeIt, graph ())];

	  std::cout << "--- edge ---" << std::endl
		    << "source position: "
		    << source.position[0] << " "
		    << source.position[1] << " "
		    << source.position[2] << std::endl
		    << "target position: "
		    << target.position[0] << " "
		    << target.position[1] << " "
		    << target.position[2] << std::endl;

	  edge.weight = (source.position - target.position).squaredNorm ();
	  if (edge.weight == 0.)
	    edge.weight = 1.;
	  else
	    edge.weight = 1. / edge.weight;
	  weightSum += edge.weight;
	}

      // Normalize weights.
      if (weightSum > 0.)
	{
	  boost::tie (edgeIt, edgeEnd) = boost::edges (graph ());
	  for (; edgeIt != edgeEnd; ++edgeIt)
	    graph ()[*edgeIt].weight /= weightSum;
	}
    }

    Eigen::Matrix<double, Eigen::Dynamic, 1>
    InteractionMesh::optimizationVector ()
    {
      Eigen::Matrix<double, Eigen::Dynamic, 1> x (optimizationVectorSize ());

      vertex_iterator_t vertexIt;
      vertex_iterator_t vertexEnd;

      boost::tie (vertexIt, vertexEnd) = boost::vertices (graph ());
      for (; vertexIt != vertexEnd; ++vertexIt)
	{
	  const Vertex& vertex = graph ()[*vertexIt];
	  x[vertex.id * 3 + 0] = vertex.position[0];
	  x[vertex.id * 3 + 1] = vertex.position[1];
	  x[vertex.id * 3 + 2] = vertex.position[2];
	}
      return x;
    }

    void operator >> (const YAML::Node& node, InteractionMesh& mesh)
    {
      if (node.Type () != YAML::NodeType::Sequence)
	{
	  std::cerr << "invalid node type" << std::endl;
	  return;
	}

      unsigned id = 0;

      // Iterate over vertices.
      for(YAML::Iterator it = node.begin (); it != node.end (); ++it)
	{
	  const YAML::Node& vertexNode = *it;

	  if (vertexNode.Type () != YAML::NodeType::Sequence)
	    {
	      std::cerr << "invalid node type" << std::endl;
	      continue;
	    }


	  InteractionMesh::vertex_descriptor_t
	    vertex = boost::add_vertex (mesh.graph ());

	  double x = 0., y = 0., z = 0.;
	  vertexNode[0] >> x;
	  vertexNode[1] >> y;
	  vertexNode[2] >> z;
	  
	  mesh.graph ()[vertex].id = id++;
	  mesh.graph ()[vertex].position[0] = x;
	  mesh.graph ()[vertex].position[1] = y;
	  mesh.graph ()[vertex].position[2] = z;
	}
      mesh.computeVertexWeights ();
    }

    InteractionMeshShPtr_t
    InteractionMesh::loadMesh (const std::string& file,
			       unsigned frameId)
    {
      std::cout << "loading mesh from file: " << file << std::endl;

      InteractionMeshShPtr_t mesh =
	boost::make_shared<InteractionMesh> ();

      std::ifstream fin (file.c_str ());
      if (!fin.good ())
	std::cerr << "bad stream"  << std::endl;
      YAML::Parser parser (fin);
      
      YAML::Node doc;

      if (!parser.GetNextDocument (doc))
	std::cerr << "empty document" << std::endl;

      if (doc.Type () != YAML::NodeType::Map)
	std::cerr << "bad node type, should be map but is "
		  << doc.Type () << std::endl;
      
      std::string type;
      doc["type"] >> type;
      if (type != "MultiVector3Seq")
	std::cerr << "bad content" << std::endl;
      // content
      // frameRate
      // numFrames

      doc["frames"][frameId] >> *mesh;

      if (parser.GetNextDocument(doc))
	{
	  std::cerr << "warning: ignoring multiple documents in YAML file"
		    << std::endl;
	}

      return mesh;
    }

    InteractionMeshShPtr_t
    InteractionMesh::makeFromOptimizationVariables
    (const Eigen::Matrix<double, Eigen::Dynamic, 1>& x)
    {
      InteractionMeshShPtr_t mesh =
	boost::make_shared<InteractionMesh> ();

      unsigned id = 0;
      for (unsigned i = 0; i < x.size () / 3; ++i)
	{
	  vertex_descriptor_t vertex =
	    boost::add_vertex (mesh->graph ());
	  
	  mesh->graph ()[vertex].id = id++;
	  mesh->graph ()[vertex].position[0] = x[i * 3 + 0];
	  mesh->graph ()[vertex].position[1] = x[i * 3 + 1];
	  mesh->graph ()[vertex].position[2] = x[i * 3 + 2];
	}
      mesh->computeVertexWeights ();
      return mesh;
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
