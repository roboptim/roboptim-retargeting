// Copyright (C) 2014 by Thomas Moulard, AIST, CNRS.
//
// This file is part of the roboptim.
//
// roboptim is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// roboptim is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with roboptim.  If not, see <http://www.gnu.org/licenses/>.

#include <fstream>

#include "path.hh"
#include "directories.hh"
#include <iostream>
namespace roboptim
{
  namespace retargeting
  {
    void resolvePath (std::string& path)
    {
      // No path, do nothing.
      if (path.empty ())
	return;

      // Path is already valid, do nothing.
      {
	std::ifstream file (path.c_str ());
	if (file.good ())
	  return;
      }

      // Path is valid, build the new path.
      std::string newPath = PKG_SHARE_DIR;

      if (path[0] != '/')
	newPath += '/';
      newPath += path;

      // And check it, if valid, update path.
      std::ifstream file (newPath.c_str ());
      if (file.good ())
	path = newPath;
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.
