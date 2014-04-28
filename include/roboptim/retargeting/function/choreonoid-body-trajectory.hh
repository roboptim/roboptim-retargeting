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

#ifndef ROBOPTIM_RETARGETING_CHOREONOID_BODY_MOTION_HH
# define ROBOPTIM_RETARGETING_CHOREONOID_BODY_MOTION_HH
# include <boost/array.hpp>
# include <boost/shared_ptr.hpp>
# include <roboptim/trajectory/vector-interpolation.hh>

# include <cnoid/BodyMotion>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Discrete trajectory built from a cnoid::BodyMotion
    ///        object
    class ChoreonoidBodyTrajectory :
      public VectorInterpolation
    {
    public:
      ROBOPTIM_TWICE_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS
      (VectorInterpolation);
      ROBOPTIM_IMPLEMENT_CLONE (ChoreonoidBodyTrajectory);

      explicit ChoreonoidBodyTrajectory (cnoid::BodyMotionPtr bodyMotion,
					 bool addFreeFloating) throw ()
	: VectorInterpolation
	  (computeParametersFromBodyMotion (bodyMotion, addFreeFloating),
	   (addFreeFloating ? 6 : 0) + bodyMotion->numJoints (),
	   1. / bodyMotion->frameRate ()),
	  bodyMotion_ (bodyMotion)
      {
      }

      virtual ~ChoreonoidBodyTrajectory () throw ()
      {}

    private:
      static vector_t computeParametersFromBodyMotion
      (cnoid::BodyMotionPtr bodyMotion, bool addFreeFloating)
      {
	vector_t::Index freeFloatingOffset = 0;
	if (addFreeFloating)
	  freeFloatingOffset = 6;
	vector_t x
	  (bodyMotion->getNumFrames ()
	   * (freeFloatingOffset + bodyMotion->numJoints ()));
	x.setZero ();
	for (int frameId = 0; frameId < bodyMotion->getNumFrames (); ++frameId)
	  {
	    if (addFreeFloating &&
		frameId < bodyMotion->linkPosSeq ()->numFrames ())
	      {
		if (!bodyMotion->linkPosSeq ())
		  throw std::runtime_error ("no linkPosSeq");
		const cnoid::MultiSE3Seq::Frame& frame =
		  bodyMotion->linkPosSeq ()->frame (frameId);

		// work around ugly API :(
		if (!&frame)
		  throw std::runtime_error ("invalid link frame");

		x.segment
		  (frameId * (freeFloatingOffset + bodyMotion->numJoints ()),
		   3) = frame[0].translation ();

		Eigen::AngleAxisd angleAxis =
		  angleAxis.fromRotationMatrix
		  (frame[0].rotation ().toRotationMatrix ());
		x.segment
		  (frameId * (freeFloatingOffset + bodyMotion->numJoints ()) + 3,
		   3) = angleAxis.angle () * angleAxis.axis ();
	      }
		if (addFreeFloating &&
		    frameId >= bodyMotion->linkPosSeq ()->numFrames ())
		  throw std::runtime_error ("no link information in MarkerMotion");

		const cnoid::MultiValueSeq::Frame& frame =
		bodyMotion->jointPosSeq ()->frame (frameId);

		// work around ugly API :(
		if (!&frame)
		  throw std::runtime_error ("invalid joint frame");

		for (int dofId = 0; dofId < bodyMotion->numJoints (); ++dofId)
		  x[frameId * (freeFloatingOffset + bodyMotion->numJoints ())
		    + freeFloatingOffset + dofId] =
		    frame[dofId];
	  }
	return x;
      }

      cnoid::BodyMotionPtr bodyMotion_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_CHOREONOID_BODY_MOTION_HH
