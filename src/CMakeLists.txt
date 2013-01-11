# Define the directory where plug-ins will be installed.
SET(PLUGINDIR lib/${PROJECT_NAME})

# Main library.
ADD_LIBRARY(roboptim-retargeting SHARED
  ${HEADERS}
  interaction-mesh.cc
  laplacian-coordinate.cc
  )
PKG_CONFIG_USE_DEPENDENCY(roboptim-retargeting roboptim-core)
SET_TARGET_PROPERTIES(roboptim-retargeting PROPERTIES SOVERSION 0.0.0)
INSTALL(TARGETS roboptim-retargeting DESTINATION lib)
