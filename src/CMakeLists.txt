# Define the directory where plug-ins will be installed.
SET(PLUGINDIR lib/${PROJECT_NAME})

# Main library.
ADD_LIBRARY(roboptim-retargeting SHARED
  ${HEADERS}
  acceleration-energy.cc
  animated-interaction-mesh.cc
  bone-length.cc
  collision.cc
  interaction-mesh.cc
  laplacian-coordinate.cc
  laplacian-deformation-energy.cc
  position.cc
  retarget.cc
  sum.cc
  torque.cc
  )
PKG_CONFIG_USE_DEPENDENCY(roboptim-retargeting roboptim-core)
PKG_CONFIG_USE_DEPENDENCY(roboptim-retargeting yaml-cpp)
SET_TARGET_PROPERTIES(roboptim-retargeting PROPERTIES SOVERSION 0.0.0)
INSTALL(TARGETS roboptim-retargeting DESTINATION lib)
