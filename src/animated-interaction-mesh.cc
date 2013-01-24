#include <algorithm>
#include <stdexcept>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>

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

  } // end of namespace retargeting.
} // end of namespace roboptim.
