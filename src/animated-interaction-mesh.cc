#include <boost/make_shared.hpp>

#include <fstream>
#include <yaml-cpp/iterator.h>
#include <yaml-cpp/yaml.h>

#include "roboptim/retargeting/animated-interaction-mesh.hh"
#include "roboptim/retargeting/interaction-mesh.hh"

namespace roboptim
{
  namespace retargeting
  {
    AnimatedInteractionMesh::AnimatedInteractionMesh ()
      :  framerate_ (),
	 meshes_ ()
    {}

    AnimatedInteractionMesh::~AnimatedInteractionMesh ()
    {}

    AnimatedInteractionMeshShPtr_t
    AnimatedInteractionMesh::loadAnimatedMesh
    (const std::string& trajectoryFile,
     const std::string& characterFile)
    {
      std::cout << "loading animated mesh from files: "
		<< trajectoryFile << " (trajectory) "
		<< characterFile << " (character)"
		<< std::endl;

      AnimatedInteractionMeshShPtr_t animatedMesh =
	boost::make_shared<AnimatedInteractionMesh> ();

      // Parse trajectory file.
      {
	std::ifstream fin (trajectoryFile.c_str ());
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
	doc["frameRate"] >> animatedMesh->framerate_;

	unsigned numFrames = 0;
	doc["numFrames"] >> numFrames;

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
	  std::cerr << "inconsistent data" << std::endl;

	if (parser.GetNextDocument(doc))
	  {
	    std::cerr << "warning: ignoring multiple documents in YAML file"
		      << std::endl;
	  }
      }

      // Parse character file.
      {
	std::ifstream fin (characterFile.c_str ());
	if (!fin.good ())
	  std::cerr << "bad stream"  << std::endl;
	YAML::Parser parser (fin);

	YAML::Node doc;

	if (!parser.GetNextDocument (doc))
	  std::cerr << "empty document" << std::endl;
	
	if (doc.Type () != YAML::NodeType::Map)
	  std::cerr << "bad node type, should be map but is "
		    << doc.Type () << std::endl;

	// Load edges.
	const YAML::Node& node = doc["edges"];
	for(YAML::Iterator it = node.begin (); it != node.end (); ++it)
	  {
	    std::string startMarker, endMarker;
	    double scale = 0.;

	    (*it)[0] >> startMarker;
	    (*it)[1] >> endMarker;
	    (*it)[2] >> scale;
	  }
      }


      return animatedMesh;
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
