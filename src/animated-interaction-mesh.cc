#include <algorithm>
#include <stdexcept>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/graph/graphviz.hpp>

#include <log4cxx/logger.h>

#include <fstream>
#include <yaml-cpp/iterator.h>
#include <yaml-cpp/yaml.h>

#include "roboptim/retargeting/animated-interaction-mesh.hh"
#include "roboptim/retargeting/interaction-mesh.hh"

#include "yaml-helper.hh"

namespace roboptim
{
  namespace retargeting
  {
    log4cxx::LoggerPtr AnimatedInteractionMesh::logger
    (log4cxx::Logger::getLogger("roboptim.retargeting.AnimatedInteractionMesh"));

    AnimatedInteractionMesh::AnimatedInteractionMesh ()
      :  framerate_ (),
	 meshes_ (),
	 vertexLabels_ ()
    {}

    AnimatedInteractionMesh::~AnimatedInteractionMesh ()
    {}

    void
    AnimatedInteractionMesh::loadEdgesFromYaml
    (const YAML::Node& node,
     AnimatedInteractionMeshShPtr_t animatedMesh)
    {
      typedef AnimatedInteractionMesh::labelsVector_t labelsVector_t;
      for(YAML::Iterator it = node.begin (); it != node.end (); ++it)
	{
	  std::string startMarker, endMarker;
	  double scale = 0.;

	  (*it)[0] >> startMarker;
	  (*it)[1] >> endMarker;
	  (*it)[2] >> scale;

	  labelsVector_t::const_iterator itStartMarker =
	    std::find (animatedMesh->vertexLabels_.begin (),
		       animatedMesh->vertexLabels_.end (),
		       startMarker);
	  labelsVector_t::const_iterator itEndMarker =
	    std::find (animatedMesh->vertexLabels_.begin (),
		       animatedMesh->vertexLabels_.end (),
		       endMarker);
	  if (itStartMarker == animatedMesh->vertexLabels_.end ())
	    {
	      LOG4CXX_WARN
		(logger,
		 boost::format("unknown marker '%1%' in character file")
		 % startMarker);
	      continue;
	    }
	  if (itEndMarker == animatedMesh->vertexLabels_.end ())
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

	  BOOST_FOREACH (InteractionMeshShPtr_t mesh,
			 animatedMesh->meshes_)
	    {
	      InteractionMesh::vertex_descriptor_t source =
		itStartMarker - animatedMesh->vertexLabels_.begin ();
	      InteractionMesh::vertex_descriptor_t target =
		itEndMarker - animatedMesh->vertexLabels_.begin ();
	      InteractionMesh::edge_descriptor_t edge;
	      bool ok;
	      boost::tie (edge, ok) =
		boost::add_edge (source, target, mesh->graph ());
	      if (!ok)
		LOG4CXX_WARN (logger, "failed to add edge");
	      mesh->graph ()[edge].scale = scale;
	      mesh->computeVertexWeights ();
	    }
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

	unsigned numFrames = 0;
	doc["numFrames"] >> numFrames;

	for (YAML::Iterator it = doc["partLabels"].begin ();
	     it != doc["partLabels"].end (); ++it)
	  {
	    std::string label;
	    *it >> label;
	    animatedMesh->vertexLabels_.push_back (label);
	  }

	for (YAML::Iterator it = doc["frames"].begin ();
	     it != doc["frames"].end (); ++it)
	  {
	    InteractionMeshShPtr_t mesh =
	      boost::make_shared<InteractionMesh> ();
	    const YAML::Node& node = *it;
	    node >> *mesh;
	    animatedMesh->meshes_.push_back (mesh);
	  }

	if (numFrames != animatedMesh->meshes_.size ())
	  throw std::runtime_error
	    ("number of frames does not match numFrames header value");

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


      return animatedMesh;
    }

    void
    AnimatedInteractionMesh::writeGraphvizGraphs (const std::string& path)
    {
      for (unsigned i = 0; i < meshes ().size (); ++i)
	{
	  std::ofstream graphvizFile
	    ((boost::format ("%1%/graph_%2%.dot")
	      % path % i).str().c_str ());
	  boost::write_graphviz
	    (graphvizFile, meshes ()[i]->graph (),
	     roboptim::retargeting::InteractionMeshGraphVertexWriter<
	       roboptim::retargeting::InteractionMesh::graph_t>
	     (meshes ()[i]->graph (),
	      vertexLabels ()),
	     roboptim::retargeting::InteractionMeshGraphEdgeWriter<
	       roboptim::retargeting::InteractionMesh::graph_t>
	     (meshes ()[i]->graph ()));
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
      animatedMesh->vertexLabels_ = previousAnimatedMesh->vertexLabels_;

      if (previousAnimatedMesh->meshes_.empty ())
	return animatedMesh;

      unsigned numVertices =
	previousAnimatedMesh->meshes_[0]->optimizationVectorSize ();
      for (unsigned frameId = 0;
	   frameId < previousAnimatedMesh->meshes_.size (); ++frameId)
	{
	  animatedMesh->meshes_.push_back
	    (InteractionMesh::makeFromOptimizationVariables
	     (x.segment (3 * numVertices * frameId, 3 * numVertices)));
	}

      return animatedMesh;
    }

    void
    AnimatedInteractionMesh::writeTrajectory (const std::string& filename)
    {
      YAML::Emitter out;
      out
	<< YAML::Comment("Marker motion data format version 1.0")
	<< YAML::BeginMap
	<< YAML::Key << "type" << YAML::Value << "MultiVector3Seq"
	<< YAML::Key << "content" << YAML::Value << "MarkerMotion"
	<< YAML::Key << "framerate" << YAML::Value << framerate_
	<< YAML::Key << "numFrames" << YAML::Value << meshes_.size ()
	<< YAML::Key << "partLabels" << YAML::Value
	<< YAML::BeginSeq
	     ;
      for (unsigned i = 0; i < vertexLabels_.size (); ++i)
	out << vertexLabels_[i];
      out
	<< YAML::EndSeq
	<< YAML::Key << "numParts" << YAML::Value << vertexLabels_.size ()
	<< YAML::Key << "frames" << YAML::Value
	<< YAML::BeginSeq
	;
      for (unsigned frameId = 0; frameId < meshes_.size (); ++frameId)
	{
	  out << YAML::BeginSeq;
	  Eigen::Matrix<double, 1, Eigen::Dynamic> x =
	    meshes_[frameId]->optimizationVector ();
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

  } // end of namespace retargeting.
} // end of namespace roboptim.
