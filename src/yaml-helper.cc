#include <stdexcept>
#include <boost/format.hpp>
#include "yaml-helper.hh"

namespace roboptim
{
  namespace retargeting
  {
    std::string
    yamlNodeTypeToString (const YAML::NodeType::value& nodeType)
    {
      switch (nodeType)
	{
	case YAML::NodeType::Null:
	  return "null";
	case YAML::NodeType::Scalar:
	  return "scalar";
	case YAML::NodeType::Sequence:
	  return "sequence";
	case YAML::NodeType::Map:
	  return "map";
	  default:
	    ;
	}
      throw std::runtime_error ("should never happen");
    }
    
    std::string
    unexpectedNodeTypeErrorMessage
    (const YAML::NodeType::value& nodeType,
     const YAML::NodeType::value& expectedNodeType)
    {
      return (boost::format ("bad node type, should be %2% but it is %1%")
	      % yamlNodeTypeToString (nodeType)
	      % yamlNodeTypeToString (expectedNodeType)).str ();
    }
    
    void
    checkNodeType
    (const YAML::Node& node,
     const YAML::NodeType::value& expectedType)
    {
      if (node.Type () != expectedType)
	throw std::runtime_error
	  (unexpectedNodeTypeErrorMessage
	   (node.Type (), expectedType));
    }
  }  // end of namespace retargeting.
} // end of namespace roboptim.
