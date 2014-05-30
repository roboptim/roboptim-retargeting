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

#ifndef ROBOPTIM_RETARGETING_PATH_HH
# define ROBOPTIM_RETARGETING_PATH_HH
# include <string>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Resolve the path.
    ///
    /// If the path is not valid as it is, try to prefix it by the
    /// share directory.  If the result is a valid path, update the
    /// path.
    ///
    /// This function is here to avoid having to specify full paths
    /// for provided data.
    ///
    /// \param[in,out] path Path be be resolved.
    void resolvePath (std::string& path);
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_PATH_HH
