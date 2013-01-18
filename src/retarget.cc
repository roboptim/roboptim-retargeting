#include "roboptim/retargeting/retarget.hh"


namespace roboptim
{
  namespace retargeting
  {
    Retarget::Retarget (const std::string& initialTrajectory)
      : animatedMesh_
	(AnimatedInteractionMesh::loadAnimatedMesh (initialTrajectory))
    {
    }

    Retarget::~Retarget ()
    {
    }

    void
    Retarget::solve ()
    {
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
