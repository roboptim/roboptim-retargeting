#ifndef ROBOPTIM_RETARGET_YAML_HELPER_HH
# define ROBOPTIM_RETARGET_YAML_HELPER_HH
# include <string>

# include <yaml-cpp/node.h>

namespace roboptim
{
  namespace retargeting
  {
    std::string
    yamlNodeTypeToString (const YAML::NodeType::value& nodeType);
    
    std::string
    unexpectedNodeTypeErrorMessage
    (const YAML::NodeType::value& nodeType,
     const YAML::NodeType::value& expectedNodeType);
    
    void
    checkNodeType
    (const YAML::Node& node,
     const YAML::NodeType::value& expectedNodeType);
  }  // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGET_YAML_HELPER_HH
