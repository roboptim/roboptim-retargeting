CONFIG_FILES(directories.hh)

INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}")
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")

# Main library.
ADD_LIBRARY(roboptim-retargeting SHARED
  ${HEADERS}
  exception.cc
  interaction-mesh.cc
  marker-mapping.cc
  morphing.cc
  path.cc
  io/choreonoid-body-motion.cc
  io/trc.cc
)

PKG_CONFIG_USE_DEPENDENCY(roboptim-retargeting yaml-cpp)

PKG_CONFIG_USE_DEPENDENCY(roboptim-retargeting libmocap)

PKG_CONFIG_USE_DEPENDENCY(roboptim-retargeting roboptim-core)
PKG_CONFIG_USE_DEPENDENCY(roboptim-retargeting roboptim-trajectory)

TARGET_LINK_LIBRARIES(roboptim-retargeting tet)
