# Make sure local headers are found
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/tests)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

MACRO(ROBOPTIM_RETARGETING_TEST NAME)
  ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  # Link against Boost.
  TARGET_LINK_LIBRARIES(${NAME} ${Boost_LIBRARIES})

  # roboptim-core and roboptim-trajectory
  PKG_CONFIG_USE_DEPENDENCY(${NAME} roboptim-core)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} roboptim-trajectory)

  # roboptim-retargeting
  TARGET_LINK_LIBRARIES(${NAME} roboptim-retargeting)

  TARGET_LINK_LIBRARIES(${NAME} blas lapack)

  # choreonoid
  PKG_CONFIG_USE_DEPENDENCY(${NAME} choreonoid)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} choreonoid-body-plugin)

  # ROS
  PKG_CONFIG_USE_DEPENDENCY(${NAME} rosconsole)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} roscpp)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} sensor_msgs)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} visualization_msgs)

  # Misc.
  TARGET_LINK_LIBRARIES(${NAME} blas lapack
    CnoidBody CnoidMocap CnoidMocapPlugin)
ENDMACRO()

SET(TESTS_DATA_DIR "${CMAKE_SOURCE_DIR}/share/roboptim/retargeting")
CONFIG_FILES(tests-config.h)

ROBOPTIM_RETARGETING_TEST(laplacian-coordinate-choreonoid)

ROBOPTIM_RETARGETING_TEST(joint-problem)

ROBOPTIM_RETARGETING_TEST(minimum-jerk)
ROBOPTIM_RETARGETING_TEST(minimum-jerk-problem)

ROBOPTIM_RETARGETING_TEST(torque-choreonoid)
ROBOPTIM_RETARGETING_TEST(torque-metapod)

ROBOPTIM_RETARGETING_TEST(zmp-choreonoid)
ROBOPTIM_RETARGETING_TEST(zmp-metapod)

ROBOPTIM_RETARGETING_TEST(forward-geometry-choreonoid)

ROBOPTIM_RETARGETING_TEST(choreonoid-body-trajectory)

ROBOPTIM_RETARGETING_TEST(body-laplacian-deformation-energy)

ROBOPTIM_RETARGETING_TEST(joint-to-marker-choreonoid)
