# Make sure local headers are found
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/tests)

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

MACRO(ROBOPTIM_RETARGETING_TEST NAME)
  ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})
  
  # Link against Boost.
  TARGET_LINK_LIBRARIES(${NAME} ${Boost_LIBRARIES})

  # Link againsts ROS.
  ROSPACK_USE_DEPENDENCY(${NAME} urdf_interface)
  ROSPACK_USE_DEPENDENCY(${NAME} urdf_parser)
  ROSPACK_USE_DEPENDENCY(${NAME} roscpp)

  # roboptim-core
  PKG_CONFIG_USE_DEPENDENCY(${NAME} roboptim-core)

  # roboptim-retargeting
  TARGET_LINK_LIBRARIES(${NAME} roboptim-retargeting)
ENDMACRO()

SET(TESTS_DATA_DIR "${CMAKE_SOURCE_DIR}/share/roboptim/retargeting")
CONFIG_FILES(tests-config.h)

ROBOPTIM_RETARGETING_TEST(bone-length)
ROBOPTIM_RETARGETING_TEST(acceleration-energy)
ROBOPTIM_RETARGETING_TEST(laplacian-coordinate)
ROBOPTIM_RETARGETING_TEST(laplacian-deformation-energy)
ROBOPTIM_RETARGETING_TEST(animated-interaction-mesh)
ROBOPTIM_RETARGETING_TEST(retarget)
ROBOPTIM_RETARGETING_TEST(inverse-kinematics)
