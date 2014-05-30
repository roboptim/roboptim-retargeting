# Copyright 2010, Thomas Moulard, LAAS-CNRS
#
# This file is part of roboptim-core.  roboptim-core is free software:
# you can redistribute it and/or modify it under the terms of the GNU
# Lesser General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any
# later version.
#
# roboptim-core is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with roboptim-core.  If not, see <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/ros.cmake)

SET(PROJECT_NAME roboptim-retargeting)
SET(PROJECT_DESCRIPTION "RobOptim retargeting")
SET(PROJECT_URL "http://github.com/thomas-moulard/roboptim-retargeting")

SET(PKG_CONFIG_ADDITIONAL_VARIABLES plugindir ${PKG_CONFIG_ADDITIONAL_VARIABLES})

SET(CSD ${CMAKE_SOURCE_DIR})
SET(HEADERS
${CSD}/include/roboptim/retargeting/function/body-laplacian-deformation-energy/choreonoid.hh
${CSD}/include/roboptim/retargeting/function/torque.hh
${CSD}/include/roboptim/retargeting/function/zmp/inverted-pendulum.hh
${CSD}/include/roboptim/retargeting/function/zmp/choreonoid.hh
${CSD}/include/roboptim/retargeting/function/zmp/metapod.hh
${CSD}/include/roboptim/retargeting/function/minimum-jerk-trajectory.hxx
${CSD}/include/roboptim/retargeting/function/torque/choreonoid.hh
${CSD}/include/roboptim/retargeting/function/torque/metapod.hh
${CSD}/include/roboptim/retargeting/function/forward-geometry/choreonoid.hh
${CSD}/include/roboptim/retargeting/function/zmp.hh
${CSD}/include/roboptim/retargeting/function/joint-to-marker/choreonoid.hh
${CSD}/include/roboptim/retargeting/function/laplacian-coordinate/choreonoid.hh
${CSD}/include/roboptim/retargeting/function/acceleration.hh
${CSD}/include/roboptim/retargeting/function/choreonoid-body-trajectory.hh
${CSD}/include/roboptim/retargeting/function/forward-geometry.hh
${CSD}/include/roboptim/retargeting/function/cost-reference-trajectory.hh
${CSD}/include/roboptim/retargeting/function/marker-laplacian-deformation-energy/choreonoid.hh
${CSD}/include/roboptim/retargeting/function/minimum-jerk-trajectory.hh
${CSD}/include/roboptim/retargeting/eigen-rigid-body.hh
${CSD}/include/roboptim/retargeting/eigen-rigid-body.hxx
)

SETUP_PROJECT()

# Add main library to pkg-config file.
PKG_CONFIG_APPEND_LIBS(roboptim-retargeting)

# Search for dependencies.
SET(BOOST_COMPONENTS date_time filesystem system
  thread program_options unit_test_framework)
SEARCH_FOR_BOOST()

ADD_REQUIRED_DEPENDENCY("libmocap")

ADD_REQUIRED_DEPENDENCY("roboptim-core >= 0.5")
ADD_REQUIRED_DEPENDENCY("roboptim-core-plugin-ipopt >= 0.5")
ADD_REQUIRED_DEPENDENCY("roboptim-trajectory >= 0.5")

ADD_REQUIRED_DEPENDENCY("choreonoid")
ADD_REQUIRED_DEPENDENCY("choreonoid-body-plugin")

ADD_REQUIRED_DEPENDENCY("yaml-cpp")

HEADER_INSTALL("${HEADERS}")

# Allow the inclusion of private headers.
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/src")

# Order matters here: the library (in src) _must_ be compiled before
# the cli tools and the tests.
ADD_SUBDIRECTORY(share)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(bin)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
