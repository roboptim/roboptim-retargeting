// Copyright (C) 2014 by Thomas Moulard, AIST, CNRS.
//
// This file is part of the roboptim.
//
// roboptim is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// roboptim is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with roboptim.  If not, see <http://www.gnu.org/licenses/>.
#include <fstream>

#include <boost/format.hpp>

#include <yaml-cpp/yaml.h>

#include <roboptim/retargeting/morphing.hh>

namespace roboptim
{
  namespace retargeting
  {
    MorphingData loadMorphingData (const std::string& filename)
    {
      MorphingData result;

      std::ifstream file (filename.c_str ());

      if (!file.good ())
	throw std::runtime_error
	  ((boost::format ("failed to open %s") % filename).str ());

      YAML::Node doc;
      YAML::Parser parser (file);
      if (!parser.GetNextDocument(doc))
	throw std::runtime_error ("failed to retrieve document");

      if (doc.Type () != YAML::NodeType::Map)
	throw std::runtime_error ("root element should be a map");

      // Parse header.
      std::string type;
      std::string version;

      if (doc["format"].Type () != YAML::NodeType::Map)
	throw std::runtime_error ("format element should be a map");
      doc["format"]["type"] >> type;
      doc["format"]["version"] >> version;

      if (type != "morphing")
	throw std::runtime_error
	  ("YAML document does not contain morphing information");
      if (version != "1.0.0" && version != "1.0" && version != "1")
	throw std::runtime_error
	  ("unsupported morphing version (should be 1.0)");

      // Parse mapping.
      if (doc["mapping"].Type () != YAML::NodeType::Map)
	throw std::runtime_error ("mapping element should be a map");

      const YAML::Node& mapping = doc["mapping"];

      for(YAML::Iterator it = mapping.begin (); it != mapping.end (); ++it)
	{
	  std::string bodyName = it.first ().to<std::string> ();

	  if (it.second ().Type () != YAML::NodeType::Sequence)
	    throw std::runtime_error
	      ("mapping value should contain sequences (arrays)");

	  for(YAML::Iterator itMarker = it.second ().begin ();
	      itMarker != it.second ().end (); ++itMarker)
	    {
	      if (itMarker->size () != 2)
		throw std::runtime_error
		  ((boost::format
		    ("2 elements were expected during YAML parsing "
		     "but only %d was found (offset are not optionals)")
		    % itMarker->size ()).str ());
	      if ((*itMarker)[1].size () != 3)
		throw std::runtime_error
		  ((boost::format
		    ("3 elements were expected during YAML parsing "
		     "but only %d was found (offset are not optionals)")
		    % (*itMarker)[1].size ()).str ());

	      MorphingDataMapping mapping;
	      mapping.marker = (*itMarker)[0].to<std::string> ();
	      mapping.offset <<
		(*itMarker)[1][0].to<double> (),
		(*itMarker)[1][1].to<double> (),
		(*itMarker)[1][2].to<double> ();
	      result.mapping[bodyName].push_back (mapping);
	    }
	}

      return result;
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.
