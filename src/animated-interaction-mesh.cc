#include <algorithm>
#include <stdexcept>
#include <fstream>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/graphviz.hpp>

#include <log4cxx/logger.h>

#include <yaml-cpp/iterator.h>
#include <yaml-cpp/yaml.h>

#include "roboptim/retargeting/animated-interaction-mesh.hh"

#include "yaml-helper.hh"

namespace roboptim
{
  namespace retargeting
  {
    log4cxx::LoggerPtr AnimatedInteractionMesh::logger
    (log4cxx::Logger::getLogger("roboptim.retargeting.AnimatedInteractionMesh"));

    AnimatedInteractionMesh::AnimatedInteractionMesh ()
      :  framerate_ (),
	 numFrames_ (),
	 numVertices_ (),
	 graph_ ()
    {}

    AnimatedInteractionMesh::~AnimatedInteractionMesh ()
    {}

    AnimatedInteractionMesh::vertex_iterator_t
    AnimatedInteractionMesh::getVertexFromLabel (const std::string& label) const
    {
      vertex_iterator_t vertexIt;
      vertex_iterator_t vertexEnd;
      boost::tie (vertexIt, vertexEnd) = boost::vertices (graph ());
      for (; vertexIt != vertexEnd; ++vertexIt)
	if (graph ()[*vertexIt].label == label)
	  return vertexIt;
      return vertexIt;
    }

    void
    AnimatedInteractionMesh::loadEdgesFromYaml
    (const YAML::Node& node,
     AnimatedInteractionMeshShPtr_t animatedMesh)
    {
      vertex_iterator_t vertexBegin;
      vertex_iterator_t vertexEnd;
      boost::tie (vertexBegin, vertexEnd) =
	boost::vertices (animatedMesh->graph ());

      typedef AnimatedInteractionMesh::labelsVector_t labelsVector_t;
      for(YAML::Iterator it = node.begin (); it != node.end (); ++it)
	{
	  std::string startMarker, endMarker;
	  double scale = 0.;

	  (*it)[0] >> startMarker;
	  (*it)[1] >> endMarker;
	  (*it)[2] >> scale;

	  vertex_iterator_t itStartMarker =
	     animatedMesh->getVertexFromLabel (startMarker);
	  vertex_iterator_t itEndMarker =
	    animatedMesh->getVertexFromLabel (endMarker);
	  if (itStartMarker == vertexEnd)
	    {
	      LOG4CXX_WARN
		(logger,
		 boost::format("unknown marker '%1%' in character file")
		 % startMarker);
	      continue;
	    }
	  if (itEndMarker == vertexEnd)
	    {
	      LOG4CXX_WARN
		(logger,
		 boost::format("unknown marker '%1%' in character file")
		 % endMarker);
	      continue;
	    }
	  if (itStartMarker == itEndMarker)
	    {
	      LOG4CXX_WARN
		(logger,
		 "source and target vertex are the same, ignoring");
	      continue;
	    }

	  edge_descriptor_t edge;
	  bool ok;
	  boost::tie (edge, ok) =
	    boost::add_edge (*itStartMarker, *itEndMarker,
			     animatedMesh->graph ());
	  if (!ok)
	    LOG4CXX_WARN (logger, "failed to add edge");
	  animatedMesh->graph ()[edge].scale = scale;
	}
    }

    AnimatedInteractionMeshShPtr_t
    AnimatedInteractionMesh::loadAnimatedMesh
    (const std::string& trajectoryFile,
     const std::string& characterFile)
    {
      LOG4CXX_INFO
	(logger,
	 "loading animated mesh from files: "
	 << trajectoryFile << " (trajectory) "
	 << characterFile << " (character)");

      AnimatedInteractionMeshShPtr_t animatedMesh =
	boost::make_shared<AnimatedInteractionMesh> ();

      // Parse trajectory file.
      {
	std::ifstream fin (trajectoryFile.c_str ());
	if (!fin.good ())
	  throw std::runtime_error ("bad stream");
	YAML::Parser parser (fin);

	YAML::Node doc;

	if (!parser.GetNextDocument (doc))
	  throw std::runtime_error ("empty document");

	checkNodeType (doc, YAML::NodeType::Map);

	std::string type;
	doc["type"] >> type;
	if (type != "MultiVector3Seq")
	  throw std::runtime_error ("bad content");
	// content
	doc["frameRate"] >> animatedMesh->framerate_;

	doc["numFrames"] >> animatedMesh->numFrames_;

	// Add one vertex per label.
	animatedMesh->numVertices_ = 0;
	for (YAML::Iterator it = doc["partLabels"].begin ();
	     it != doc["partLabels"].end (); ++it)
	  {
	    std::string label;
	    *it >> label;

	    vertex_descriptor_t
	      vertex = boost::add_vertex (animatedMesh->graph ());

	    animatedMesh->numVertices_++;
	    animatedMesh->graph ()[vertex].label = label;
	    animatedMesh->graph ()[vertex].positions.resize
	      (animatedMesh->numFrames_);
	  }

	unsigned frameId = 0;
	for (YAML::Iterator it = doc["frames"].begin ();
	     it != doc["frames"].end (); ++it)
	  {
	    const YAML::Node& node = *it;
	    checkNodeType (node, YAML::NodeType::Sequence);

	    if (frameId >= animatedMesh->numFrames_)
	      throw std::runtime_error
		("announced number of frames do not match data");

	    // Iterate over vertices.
	    vertex_descriptor_t vertex = 0;
	    for(YAML::Iterator it = node.begin (); it != node.end (); ++it)
	      {
		const YAML::Node& vertexNode = *it;

		checkNodeType (vertexNode, YAML::NodeType::Sequence);

		if (vertex >=
		    animatedMesh->numVertices_)
		  throw std::runtime_error
		    ("announced number of vertices do not match data");

		double x = 0., y = 0., z = 0.;
		vertexNode[0] >> x;
		vertexNode[1] >> y;
		vertexNode[2] >> z;

		animatedMesh->graph ()[vertex].positions[frameId][0] = x;
		animatedMesh->graph ()[vertex].positions[frameId][1] = y;
		animatedMesh->graph ()[vertex].positions[frameId][2] = z;
		++vertex;
	      }
	    ++frameId;
	  }

	if (parser.GetNextDocument(doc))
	  LOG4CXX_WARN (logger, "ignoring multiple documents in YAML file");
      }

      // Parse character file.
      {
	std::ifstream fin (characterFile.c_str ());
	if (!fin.good ())
	  throw std::runtime_error ("bad stream");
	YAML::Parser parser (fin);

	YAML::Node doc;

	if (!parser.GetNextDocument (doc))
	  throw std::runtime_error ("empty document");

	checkNodeType (doc, YAML::NodeType::Map);

	// Load edges.
	loadEdgesFromYaml (doc["edges"], animatedMesh);
	loadEdgesFromYaml (doc["extraMarkerEdges"], animatedMesh);
      }

      animatedMesh->computeVertexWeights ();
      return animatedMesh;
    }

    void
    AnimatedInteractionMesh::writeGraphvizGraphs (std::ostream& out,
						  unsigned i)
    {
      boost::write_graphviz
	(out, graph (),
	 InteractionMeshGraphVertexWriter<graph_t> (graph (), i),
	 InteractionMeshGraphEdgeWriter<graph_t> (graph (), i));

    }

    void
    AnimatedInteractionMesh::writeGraphvizGraphs (const std::string& path)
    {
      for (unsigned i = 0; i < numFrames (); ++i)
	{
	  std::ofstream graphvizFile
	    ((boost::format ("%1%/graph_%2%.dot")
	      % path % i).str().c_str ());
	  writeGraphvizGraphs (graphvizFile, i);
	}
    }

    AnimatedInteractionMeshShPtr_t
    AnimatedInteractionMesh::makeFromOptimizationVariables
    (const Eigen::Matrix<double, Eigen::Dynamic, 1>& x,
     AnimatedInteractionMeshShPtr_t previousAnimatedMesh)
    {
      AnimatedInteractionMeshShPtr_t animatedMesh =
	boost::make_shared<AnimatedInteractionMesh> ();

      animatedMesh->framerate_ = previousAnimatedMesh->framerate_;
      animatedMesh->numVertices_ = previousAnimatedMesh->numVertices_;
      animatedMesh->numFrames_ = previousAnimatedMesh->numFrames_;

      boost::copy_graph (previousAnimatedMesh->graph (),
			 animatedMesh->graph_);

      // Update positions using optimization vector.
      vertex_iterator_t vertexIt;
      vertex_iterator_t vertexEnd;
      boost::tie (vertexIt, vertexEnd) = boost::vertices
	(animatedMesh->graph ());
      unsigned vertexId = 0;
      for (; vertexIt != vertexEnd; ++vertexIt)
	{
	  for (unsigned frameId = 0;
	       frameId < animatedMesh->numFrames_; ++frameId)
	    animatedMesh->graph ()[*vertexIt].positions[frameId] =
	      x.segment
	      (frameId * animatedMesh->numVertices_ * 3 + vertexId * 3, 3);
	  ++vertexId;
	}

      // Update weights.
      animatedMesh->computeVertexWeights ();
      
      return animatedMesh;
    }

    void
    AnimatedInteractionMesh::writeTrajectory (const std::string& filename)
    {
      YAML::Emitter out;
      vertex_iterator_t vertexIt;
      vertex_iterator_t vertexEnd;

      out
	<< YAML::Comment("Marker motion data format version 1.0")
	<< YAML::BeginMap
	<< YAML::Key << "type" << YAML::Value << "MultiVector3Seq"
	<< YAML::Key << "content" << YAML::Value << "MarkerMotion"
	<< YAML::Key << "framerate" << YAML::Value << framerate_
	<< YAML::Key << "numFrames" << YAML::Value << numFrames_
	<< YAML::Key << "partLabels" << YAML::Value
	<< YAML::BeginSeq
	     ;
      boost::tie (vertexIt, vertexEnd) = boost::vertices (graph ());
      for (; vertexIt != vertexEnd; ++vertexIt)
	out << graph ()[*vertexIt].label;
      out
	<< YAML::EndSeq
	<< YAML::Key << "numParts" << YAML::Value << numVertices_
	<< YAML::Key << "frames" << YAML::Value
	<< YAML::BeginSeq
	;
      for (unsigned frameId = 0; frameId < numFrames_; ++frameId)
	{
	  out << YAML::BeginSeq;
	  Eigen::VectorXd x = makeOptimizationVectorOneFrame (frameId);
	  for (unsigned i = 0; i < x.cols (); ++i)
	    out << x[i];
	  out << YAML::EndSeq;
	}
      out
	<< YAML::EndSeq
	<< YAML::EndMap
	;
      
      std::ofstream file (filename.c_str ());
      file << out.c_str ();
    }

    Eigen::VectorXd
    AnimatedInteractionMesh::makeOptimizationVector () const
    {
      Eigen::VectorXd x (optimizationVectorSize ());
      x.setZero ();
      if (!numFrames_ || !numVertices_)
	return x;

      unsigned length = optimizationVectorSizeOneFrame ();
      unsigned idx = 0;
      for (unsigned i = 0; i < numFrames_; ++i)
	{
	  x.segment (idx, length) = makeOptimizationVectorOneFrame (i);
	  idx += length;
	}

      return x;
    }

    Eigen::VectorXd
    AnimatedInteractionMesh::makeOptimizationVectorOneFrame
    (unsigned frameId) const
    {
      Eigen::VectorXd
	x (optimizationVectorSizeOneFrame ());
      x.setZero ();
      if (!numVertices_)
	return x;

      unsigned idx = 0;

      vertex_iterator_t vertexIt;
      vertex_iterator_t vertexEnd;
      boost::tie (vertexIt, vertexEnd) = boost::vertices (graph ());
      for (; vertexIt != vertexEnd; ++vertexIt)
	{
	  x.segment (idx, 3) = graph ()[*vertexIt].positions[frameId];
	  idx += 3;
	}

      return x;
    }

    void
    AnimatedInteractionMesh::computeVertexWeights ()
    {
      edge_iterator_t edgeIt;
      edge_iterator_t edgeEnd;

      for (unsigned frameId = 0; frameId < numFrames_; ++frameId)
	{
	  double weightSum = 0.;
	  boost::tie (edgeIt, edgeEnd) = boost::edges (graph ());
	  for (; edgeIt != edgeEnd; ++edgeIt)
	    {
	      Edge& edge = graph ()[*edgeIt];
	      const Vertex& source =
		graph ()[boost::source (*edgeIt, graph ())];
	      const Vertex& target =
		graph ()[boost::target (*edgeIt, graph ())];

	      LOG4CXX_TRACE(logger,
			    "--- edge ---\n"
			    << "source position: "
			    << source.positions[frameId][0] << " "
			    << source.positions[frameId][1] << " "
			    << source.positions[frameId][2] << "\n"
			    << "target position: "
			    << target.positions[frameId][0] << " "
			    << target.positions[frameId][1] << " "
			    << target.positions[frameId][2]);

	      if (edge.weight.size () != numFrames_)
		edge.weight.resize (numFrames_);

	      edge.weight[frameId] =
		(source.positions[frameId] - target.positions[frameId]
		 ).squaredNorm ();
	      if (edge.weight[frameId] == 0.)
		edge.weight[frameId] = 1.;
	      else
		edge.weight[frameId] = 1. / edge.weight[frameId];
	      weightSum += edge.weight[frameId];
	    }

	  // Normalize weights.
	  if (weightSum > 0.)
	    {
	      boost::tie (edgeIt, edgeEnd) = boost::edges (graph ());
	      for (; edgeIt != edgeEnd; ++edgeIt)
		graph ()[*edgeIt].weight[frameId] /= weightSum;
	    }
	}
    }

    void
    AnimatedInteractionMesh::recomputeCachedData ()
    {
      numVertices_ = boost::num_vertices (graph ());
      numFrames_ = 0;
      if (!numVertices_)
	return;
      vertex_descriptor_t v = 0;
      numFrames_ = graph ()[v].positions.size ();
      computeVertexWeights ();
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
