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

#ifndef ROBOPTIM_RETARGETING_IO_CHOREONOID_BODY_MOTION_HH
# define ROBOPTIM_RETARGETING_IO_CHOREONOID_BODY_MOTION_HH
# include <string>

# include <boost/shared_ptr.hpp>

# include <roboptim/trajectory/trajectory.hh>

# include <roboptim/retargeting/config.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Write a trajectory as TRC file.
    ///
    /// \param[in] filename output filename
    /// \param[in] trajectory trajectory to be written
    ROBOPTIM_RETARGETING_DLLEXPORT void
    writeBodyMotion (const std::string& filename,
		     boost::shared_ptr<roboptim::Trajectory<3> > result);
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_IO_CHOREONOID_BODY_MOTION_HH
