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

#ifndef ROBOPTIM_RETARGETING_TRC_HH
# define ROBOPTIM_RETARGETING_TRC_HH
# include <string>

# include <boost/shared_ptr.hpp>

# include <roboptim/trajectory/trajectory.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Write a trajectory as TRC file.
    ///
    /// \param[in] filename output filename
    /// \param[in] trajectory trajectory to be written
    void writeTRC
    (const std::string& filename,
     const roboptim::Trajectory<3>& trajectory);
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_TRC_HH
