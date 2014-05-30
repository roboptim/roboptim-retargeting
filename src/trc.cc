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
#include <boost/format.hpp>

#include "trc.hh"

namespace roboptim
{
  namespace retargeting
  {
    void writeTRC (const std::string& filename,
		   const roboptim::Trajectory<3>& trajectory)
    {
      typedef roboptim::Trajectory<3>::value_type value_type;

      std::ofstream file (filename.c_str ());

      std::string headerFormatStr =
	"PathFileType	%d	%s	%s\n"
	"DataRate	CameraRate	NumFrames	NumMarkers	Units	OrigDataRate	OrigDataStartFrame	OrigNumFrames\n"
	"%f	%f	      %d	%d	%s	%f	%d	      %d\n"
	"Frame#	Time	box1			box2			box3			box4\n"
	"X1	Y1	Z1	X2	Y2	Z2	X3	Y3	Z3	X4	Y4	Z4\n";

      typedef roboptim::Function::vector_t::Index index_t;
      index_t numMarkers =
	static_cast<index_t> (trajectory.outputSize () / 3);
      index_t numFrames =
	static_cast<index_t>
	(trajectory.length () / static_cast<value_type> (trajectory.outputSize ()));
      index_t dataRate = static_cast<index_t>
	(trajectory.length () / static_cast<value_type> (numFrames));

      boost::format headerFormat (headerFormatStr);
      headerFormat
	% 4
	% "(X/Y/Z)"
	% filename
	% dataRate // Data rate
	% dataRate // Camera rate
	% numFrames // Num Frames
	% numMarkers // Num Markers
	% "m" // Units
	% dataRate // OrigDataRate
	% 1 // OrigDataStartFrame
	% numFrames // OrigNumFrames
	;

      file << headerFormat.str ();

      for (index_t frameId = 0; frameId < numFrames; ++frameId)
	{
	  file << (frameId + 1) << " ";
	  for (index_t markerId = 0; markerId < numMarkers; ++markerId)
	    file
	      << trajectory.parameters ()
	      [frameId * numMarkers * 3 + markerId * 3 + 0]
	      << " "
	      << trajectory.parameters ()
	      [frameId * numMarkers * 3 + markerId * 3 + 1]
	      << " "
	      << trajectory.parameters ()
	      [frameId * numMarkers * 3 + markerId * 3 + 2]
	      << " ";
	  file << "\n";
	}
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.
