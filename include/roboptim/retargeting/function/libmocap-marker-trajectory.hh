// Copyright (C) 2013 by Thomas Moulard, AIST, CNRS, INRIA.
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

#ifndef ROBOPTIM_RETARGETING_MARKER_TRAJECTORY_HH
# define ROBOPTIM_RETARGETING_MARKER_TRAJECTORY_HH
# include <boost/array.hpp>
# include <boost/shared_ptr.hpp>
# include <roboptim/trajectory/vector-interpolation.hh>
# include <roboptim/retargeting/utility.hh>

# include <cnoid/BodyMotion>

namespace roboptim
{
  namespace retargeting
  {
    ROBOPTIM_RETARGETING_PREDECLARE_CLASS (LibmocapMarkerTrajectory);

    /// \brief Discrete trajectory built from a cnoid::BodyMotion
    ///        object
    class LibmocapMarkerTrajectory : public VectorInterpolation
    {
    public:
      ROBOPTIM_TWICE_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS
      (VectorInterpolation);
      ROBOPTIM_IMPLEMENT_CLONE (LibmocapMarkerTrajectory);

      explicit LibmocapMarkerTrajectory
      (const libmocap::MarkerTrajectory& markersTrajectory)
	: VectorInterpolation
	  (computeParametersFromBodyMotion (markersTrajectory),
	   static_cast<size_type> (markersTrajectory.numMarkers () * 3),
	   1. / markersTrajectory.dataRate ())
      {}

      virtual ~LibmocapMarkerTrajectory ()
      {}

      LibmocapMarkerTrajectory
      (const LibmocapMarkerTrajectory& trajectory)
	: VectorInterpolation (trajectory)
      {}

    private:
      static vector_t computeParametersFromBodyMotion
      (const libmocap::MarkerTrajectory& markersTrajectory)
      {
	vector_t::Index oneFrameSize = markersTrajectory.numMarkers () * 3;
	vector_t parameters
	  (markersTrajectory.numFrames () * oneFrameSize);

	const std::size_t nFrames =
	  static_cast<std::size_t> (markersTrajectory.numFrames ());
	for (std::size_t frameId = 0; frameId < nFrames; ++frameId)
	  {
	    ROBOPTIM_RETARGETING_ASSERT
	      (static_cast<std::size_t> (oneFrameSize) + 1 ==
	       markersTrajectory.positions ()[frameId].size ());

	    vector_t::Index offset =
	      static_cast<vector_t::Index> (frameId) * oneFrameSize;

	    // Start from 1, 0 is time.
	    Eigen::Map<const vector_t> frame
	      (&markersTrajectory.positions ()[frameId][1], oneFrameSize);
	    parameters.segment (offset, oneFrameSize) = frame;
	  }
	return parameters;
      }
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_MARKER_TRAJECTORY_HH
