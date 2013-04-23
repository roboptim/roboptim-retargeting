CONFIG_FILES(directories.hh)
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")

# Main library.
ADD_LIBRARY(roboptim-retargeting SHARED
  ${HEADERS}
  acceleration-energy.cc
  animated-interaction-mesh.cc
  bone-length.cc
  collision.cc
  laplacian-coordinate.cc
  laplacian-deformation-energy.cc
  position.cc
  retarget.cc
  torque.cc
  zmp.cc
  yaml-helper.cc yaml-helper.hh

  model/hrp4g2.cc model/hrp4g2.hh
  )
PKG_CONFIG_USE_DEPENDENCY(roboptim-retargeting roboptim-core)
PKG_CONFIG_USE_DEPENDENCY(roboptim-retargeting metapod)
PKG_CONFIG_USE_DEPENDENCY(roboptim-retargeting yaml-cpp)
PKG_CONFIG_USE_DEPENDENCY(roboptim-retargeting liblog4cxx)
TARGET_LINK_LIBRARIES(roboptim-retargeting tet)
SET_TARGET_PROPERTIES(roboptim-retargeting PROPERTIES SOVERSION 0.0.0)
INSTALL(TARGETS roboptim-retargeting DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Main binary.
ADD_EXECUTABLE(roboptim-retarget-motion roboptim-retarget-motion)
TARGET_LINK_LIBRARIES(roboptim-retarget-motion
  roboptim-retargeting ${Boost_LIBRARIES})
INSTALL(TARGETS roboptim-retarget-motion
  DESTINATION ${CMAKE_INSTALL_BINDIR})
