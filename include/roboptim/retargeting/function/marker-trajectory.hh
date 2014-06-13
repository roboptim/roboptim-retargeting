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
    /// \brief Build a RobOptim trajectory from a libmocap object.
    ///
    /// This class also builds the MarkerMapping class storing the
    /// mapping between the markers names and their IDs (and
    /// vice-versa).
    class LibmocapMarkerTrajectory : public VectorInterpolation
    {
    public:
      ROBOPTIM_TWICE_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS
      (VectorInterpolation);
      ROBOPTIM_IMPLEMENT_CLONE (MarkerTrajectory);

      explicit LibmocapMarkerTrajectory
      (const libmocap::MarkerTrajectory& markersTrajectory)
	: VectorInterpolation
	  (computeParametersFromBodyMotion (markersTrajectory),
	   (markersTrajectory.markers ().size () * 3,
	    1. / markersTrajectory.frameRate ()))
      {}

      virtual ~LibmocapMarkerTrajectory ()
      {}

    private:
      static vector_t computeParametersFromBodyMotion
      (const libmocap::MarkerTrajectory& markersTrajectory)
      {
	vector_t parameters (markersTrajectory);

	std::size_t oneFrameSize = markersTrajectory.numMarkers () * 3;

	for (std::size_t frameId = 0;
	     frameId < markersTrajectory.numFrames (); ++frameId)
	  {
	    ROBOPTIM_RETARGETING_ASSERT
	      (oneFrameSize == markersTrajectory.positions ()[frameId].size ());
	    std::size_t offset = frameId * oneFrameSize;

	    Eigen::Map<vector_t> frame
	      (&markersTrajectory.positions ()[frameId][0], oneFrameSize);
	    parameters.segment (offset, oneFrameSize) = frame;
	  }
      }

      cnoid::BodyMotionPtr bodyMotion_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_MARKER_TRAJECTORY_HH
