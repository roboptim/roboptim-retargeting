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

SET(CXX_DISABLE_WERROR 1)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES plugindir ${PKG_CONFIG_ADDITIONAL_VARIABLES})

SET(CSD ${CMAKE_SOURCE_DIR})
SET(HEADERS
  ${CSD}/include/roboptim/retargeting/acceleration-energy.hh
  ${CSD}/include/roboptim/retargeting/animated-interaction-mesh.hh
  ${CSD}/include/roboptim/retargeting/bone-length.hh
  ${CSD}/include/roboptim/retargeting/collision.hh
  ${CSD}/include/roboptim/retargeting/laplacian-coordinate.hh
  ${CSD}/include/roboptim/retargeting/laplacian-deformation-energy.hh
  ${CSD}/include/roboptim/retargeting/position.hh
  ${CSD}/include/roboptim/retargeting/retarget.hh
  ${CSD}/include/roboptim/retargeting/sum.hh
  ${CSD}/include/roboptim/retargeting/sum.hxx
  ${CSD}/include/roboptim/retargeting/torque.hh
  )

SETUP_PROJECT()

# Add main library to pkg-config file.
PKG_CONFIG_APPEND_LIBS(roboptim-retargeting)

# Search for dependencies.
SEARCH_FOR_BOOST()

ADD_REQUIRED_DEPENDENCY("roboptim-core >= 0.5")
ADD_REQUIRED_DEPENDENCY("roboptim-core-plugin-ipopt >= 0.5")
ADD_REQUIRED_DEPENDENCY("yaml-cpp")
ADD_REQUIRED_DEPENDENCY("liblog4cxx >= 0.10.0")
ADD_REQUIRED_DEPENDENCY("metapod")

ADD_REQUIRED_DEPENDENCY("choreonoid")
ADD_REQUIRED_DEPENDENCY("choreonoid-body-plugin")

HEADER_INSTALL("${HEADERS}")

ADD_SUBDIRECTORY(share)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
