# Copyright 2010, Thomas Moulard, LAAS-CNRS
#
# This file is part of roboptim-core.
# roboptim-core is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
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
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME roboptim-trajectory)
SET(PROJECT_DESCRIPTION "RobOptim retargeting")
SET(PROJECT_URL "http://github.com/thomas-moulard/roboptim-retargeting")

SET(CSD ${CMAKE_SOURCE_DIR})
SET(HEADERS
  ${CSD}/include/roboptim/retargeting/interaction-mesh.hh
  ${CSD}/include/roboptim/retargeting/laplacian-coordinate.hh
  )

SET(CXX_DISABLE_WERROR True)

SETUP_PROJECT()

# Add main library to pkg-config file.
PKG_CONFIG_APPEND_LIBS(roboptim-retargeting)

# Search for dependencies.
SEARCH_FOR_BOOST()

ADD_REQUIRED_DEPENDENCY("roboptim-core >= 0.5")
ADD_REQUIRED_DEPENDENCY("yaml-cpp")

HEADER_INSTALL("${HEADERS}")

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
