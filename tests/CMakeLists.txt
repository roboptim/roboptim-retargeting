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
ENDMACRO()
