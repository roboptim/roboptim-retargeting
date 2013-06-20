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

#include <tetgen.h>

#include "roboptim/retargeting/animated-interaction-mesh.hh"

#include "yaml-helper.hh"

// Remove trace logging in release.
#ifdef NDEBUG
# undef LOG4CXX_TRACE
# define LOG4CXX_TRACE(logger, msg)
#endif //!NDEBUG

namespace roboptim
{
  namespace retargeting
  {
    log4cxx::LoggerPtr AnimatedInteractionMesh::logger
    (log4cxx::Logger::getLogger
     ("roboptim.retargeting.AnimatedInteractionMesh"));

    AnimatedInteractionMesh::AnimatedInteractionMesh ()
      :  framerate_ (),
	 numFrames_ (),
	 numVertices_ (),
	 state_ (),
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

    AnimatedInteractionMesh::vertex_iterator_t
    AnimatedInteractionMesh::getVertexFromPosition (unsigned frameId,
						    double x,
						    double y,
						    double z) const
    {
      vertex_iterator_t vertexIt;
      vertex_iterator_t vertexEnd;
      boost::tie (vertexIt, vertexEnd) = boost::vertices (graph ());
      for (; vertexIt != vertexEnd; ++vertexIt)
	if (graph ()[*vertexIt].positions[frameId][0] == x
	    && graph ()[*vertexIt].positions[frameId][1] == y
	    && graph ()[*vertexIt].positions[frameId][2] == z)
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
	  bool ok = false;
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
    (cnoid::MarkerMotionPtr markerMotion,
     cnoid::CharacterPtr character)
    {
      AnimatedInteractionMeshShPtr_t animatedMesh =
	boost::make_shared<AnimatedInteractionMesh> ();

      // Copy marker motion.
      animatedMesh->framerate_ = markerMotion->frameRate ();
      animatedMesh->numFrames_ = markerMotion->numFrames ();
      animatedMesh->numVertices_ = markerMotion->numParts ();

      std::cout << "frame rate: " << animatedMesh->framerate_ << std::endl;
      std::cout << "num frame: " << animatedMesh->numFrames_ << std::endl;
      std::cout << "num vertices: " << animatedMesh->numVertices_ << std::endl;

      // Resize state vector.
      animatedMesh->state_.resize
	(animatedMesh->numFrames_ * animatedMesh->numVertices_ * 3);
      animatedMesh->state_.setZero ();
      std::cout << "state size: " << animatedMesh->state_.size () << std::endl;

      // Resize interaction mesh vector.
      animatedMesh->interactionMeshes_.resize (animatedMesh->numFrames_);

      std::cout << "interaction mesh size: " << animatedMesh->interactionMeshes_.size () << std::endl;

      for (int partId = 0; partId < animatedMesh->numVertices_; ++partId)
	{
	  std::string label = markerMotion->markerLabel (partId);

	  vertex_descriptor_t
	    vertex = boost::add_vertex (animatedMesh->graph ());

	  animatedMesh->graph ()[vertex].label = label;

	  for (unsigned frameId = 0;
		 frameId < animatedMesh->numFrames_; ++frameId)
	    {
	      Eigen::VectorXd::Index offset =
		frameId * animatedMesh->numVertices_ * 3 + vertex * 3;
	      animatedMesh->graph ()[vertex].positions.push_back
		(Vertex::position_t (animatedMesh->state_, offset, 3));

	      interaction_mesh_vertex_descriptor_t
		interactionMeshVertex =
		boost::add_vertex (animatedMesh->interactionMeshes_[frameId]);
	      animatedMesh->interactionMeshes_[frameId]
		[interactionMeshVertex].label = label;
	    }
	}

      for (int frameId = 0; frameId < animatedMesh->numFrames_; ++frameId)
	{
	  if (frameId >= animatedMesh->numFrames_)
	    throw std::runtime_error
	      ("announced number of frames do not match data");

	  // Iterate over vertices.
	  vertex_descriptor_t vertex = 0;

	  for (int partId = 0; partId < animatedMesh->numVertices_; ++partId)
	    {
	      if (vertex >=
		  animatedMesh->numVertices_)
		throw std::runtime_error
		  ("announced number of vertices do not match data");
	      animatedMesh->graph ()[vertex].positions[frameId] =
		markerMotion->frame (frameId)[partId];
	      ++vertex;
	    }
	  ++frameId;
	}

      // Parse character file.
      std::string characterFile
	("/home/moulard/profiles/default-x86_64-linux-ubuntu-12.04.1/"
	 "src/unstable/aist/choreonoid/ext/MocapPlugin/TestData/"
	 "character-cmu-hrp4c.yaml");
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

      animatedMesh->computeInteractionMeshes ();
      animatedMesh->computeVertexWeights ();
      return animatedMesh;
    }

    void
    AnimatedInteractionMesh::writeGraphvizGraphs (std::ostream& out,
						  unsigned i)
    {
      boost::write_graphviz
	(out, graph (),
	 GraphVertexWriter<graph_t> (graph (), i),
	 GraphEdgeWriter<graph_t> (graph (), i));

    }

    void
    AnimatedInteractionMesh::writeGraphvizInteractionMeshGraphs
    (std::ostream& out, unsigned i)
    {
      boost::write_graphviz
      	(out, interactionMeshes ()[i],
      	 InteractionMeshGraphVertexWriter<interaction_mesh_graph_t>
      	 (interactionMeshes ()[i]),
      	 InteractionMeshGraphEdgeWriter<interaction_mesh_graph_t>
      	 (interactionMeshes ()[i]));
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

	  std::ofstream graphvizFile2
	    ((boost::format ("%1%/graph_im_%2%.dot")
	      % path % i).str().c_str ());
	  writeGraphvizInteractionMeshGraphs (graphvizFile2, i);
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
      animatedMesh->interactionMeshes_ =
	previousAnimatedMesh->interactionMeshes_;

      // This is what changes.
      animatedMesh->state_ = x;

      boost::copy_graph (previousAnimatedMesh->graph (),
			 animatedMesh->graph_);

      // Update positions using optimization vector.
      vertex_iterator_t vertexIt;
      vertex_iterator_t vertexEnd;
      boost::tie (vertexIt, vertexEnd) = boost::vertices
	(animatedMesh->graph ());
      unsigned vertexId = 0;
      for (; vertexIt != vertexEnd; ++vertexIt, ++vertexId)
	for (unsigned frameId = 0;
	     frameId < animatedMesh->numFrames_; ++frameId)
	  {
	    Eigen::VectorXd::Index offset =
	      frameId * animatedMesh->numVertices_ * 3 + vertexId * 3;
	    assert (offset <= animatedMesh->state ().size () - 3);

	    // use placement new to map to the new state
	    new (&animatedMesh->graph ()[*vertexIt].positions[frameId])
	      Vertex::position_t (animatedMesh->state_, offset, 3);
	  }

      // Update weights.
      animatedMesh->computeInteractionMeshes ();
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
	<< YAML::Key << "frameRate" << YAML::Value << framerate_
	<< YAML::Key << "numFrames" << YAML::Value << numFrames_
	<< YAML::Key << "partLabels" << YAML::Value
	<< YAML::Flow
	<< YAML::BeginSeq
	;
      boost::tie (vertexIt, vertexEnd) = boost::vertices (graph ());
      for (; vertexIt != vertexEnd; ++vertexIt)
	out << graph ()[*vertexIt].label;
      out
	<< YAML::EndSeq
	<< YAML::Key << "numParts" << YAML::Value << numVertices_
	<< YAML::Key << "frames" << YAML::Value
	<< YAML::Flow
	<< YAML::BeginSeq
	;
      for (unsigned frameId = 0; frameId < numFrames_; ++frameId)
	{
	  out << YAML::BeginSeq << YAML::Flow;
	  Eigen::VectorXd x = makeOptimizationVectorOneFrame (frameId);
	  for (unsigned i = 0; i < x.size (); i += 3)
	    {
	      out << YAML::BeginSeq;
	      out << x[i + 0];
	      out << x[i + 1];
	      out << x[i + 2];
	      out << YAML::EndSeq;
	    }
	  out << YAML::EndSeq;
	}
      out
	<< YAML::EndSeq
	<< YAML::EndMap
	;

      std::ofstream file (filename.c_str ());
      file << out.c_str ();
    }

    void
    AnimatedInteractionMesh::computeVertexWeights ()
    {
      interaction_mesh_edge_iterator_t edgeIt;
      interaction_mesh_edge_iterator_t edgeEnd;

      for (unsigned frameId = 0; frameId < numFrames_; ++frameId)
	{
	  double weightSum = 0.;
	  boost::tie (edgeIt, edgeEnd) =
	    boost::edges (interactionMeshes ()[frameId]);
	  for (; edgeIt != edgeEnd; ++edgeIt)
	    {
	      InteractionMeshEdge& edge =
		interactionMeshes ()[frameId][*edgeIt];

	      //FIXME: make the assumpation both graph have the same
	      // vertex order.
	      const Vertex& source =
		graph ()
		[boost::source (*edgeIt, interactionMeshes ()[frameId])];
	      const Vertex& target =
		graph ()
		[boost::target (*edgeIt, interactionMeshes ()[frameId])];

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

	      edge.weight =
		(source.positions[frameId] - target.positions[frameId]
		 ).squaredNorm ();
	      if (edge.weight == 0.)
		edge.weight = 1.;
	      else
		edge.weight = 1. / edge.weight;
	      weightSum += edge.weight;
	    }

	  // Normalize weights.
	  if (weightSum > 0.)
	    {
	      boost::tie (edgeIt, edgeEnd) =
		boost::edges (interactionMeshes ()[frameId]);
	      for (; edgeIt != edgeEnd; ++edgeIt)
		interactionMeshes ()[frameId][*edgeIt].weight
		  /= weightSum;
	    }
	}
    }

    void
    AnimatedInteractionMesh::recomputeCachedData ()
    {
      numVertices_ = (unsigned int)boost::num_vertices (graph ());
      numFrames_ = 0;
      if (!numVertices_)
	throw std::runtime_error ("no vertices in graph");
      vertex_descriptor_t v = 0;
      numFrames_ = (unsigned int)graph ()[v].positions.size ();
      computeInteractionMeshes ();
      computeVertexWeights ();
    }

    void
    AnimatedInteractionMesh::computeInteractionMeshes ()
    {
      for (unsigned i = 0; i < numFrames (); ++i)
	computeInteractionMesh (i);
    }

    void
    AnimatedInteractionMesh::computeInteractionMesh (unsigned frameId)
    {
      assert (frameId < interactionMeshes_.size ());
      assert (boost::num_vertices (interactionMeshes_[frameId])
	      == boost::num_vertices (graph ()));

      // Remove edges.
      {
	interaction_mesh_edge_iterator_t ei, ei_end, next;
	boost::tie (ei, ei_end) =
	  boost::edges (interactionMeshes_[frameId]);
	for (next = ei; ei != ei_end; ei = next)
	  {
	    ++next;
	    boost::remove_edge (*ei, interactionMeshes_[frameId]);
	  }
      }

      vertex_iterator_t vertexIt;
      vertex_iterator_t vertexEnd;

      tetgenio in, out;
      in.firstnumber = 0;

      in.numberofpoints = numVertices ();
      in.pointlist = new REAL[in.numberofpoints * 3];

      boost::tie (vertexIt, vertexEnd) = boost::vertices (graph ());
      unsigned j = 0;
      for (; vertexIt != vertexEnd; ++vertexIt, ++j)
	{
	  in.pointlist[j * 3 + 0] =
	    graph ()[*vertexIt].positions[frameId][0];
	  in.pointlist[j * 3 + 1] =
	    graph ()[*vertexIt].positions[frameId][1];
	  in.pointlist[j * 3 + 2] =
	    graph ()[*vertexIt].positions[frameId][2];
	}

      char switches[] = "zQ";
      tetrahedralize (switches, &in, &out);

      const int n = out.numberoftetrahedra;
      const int m = out.numberofcorners;
      for (int i = 0; i < n; ++i)
	{
	  for (int j = 0; j < m - 1; ++j)
	    {
	      for (int k = j + 1; k < m; ++k)
		{
		  int index0 = out.tetrahedronlist[i * m + j];
		  int index1 = out.tetrahedronlist[i * m + k];

		  interaction_mesh_edge_descriptor_t edge;
		  interaction_mesh_vertex_descriptor_t startMarker =
		    index0; //FIXME:
		  interaction_mesh_vertex_descriptor_t endMarker =
		    index1; //FIXME:

		  bool ok = false;
		  boost::tie (edge, ok) =
		    boost::add_edge (startMarker, endMarker,
				     interactionMeshes_[frameId]);
		}
	    }
	}
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.
