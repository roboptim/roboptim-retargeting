#include "roboptim/retargeting/retarget.hh"

#include "roboptim/retargeting/laplacian-deformation-energy.hh"


namespace roboptim
{
  namespace retargeting
  {
    Retarget::Retarget (const std::string& initialTrajectory)
      : animatedMesh_
	(AnimatedInteractionMesh::loadAnimatedMesh (initialTrajectory)),
	cost_ (animatedMesh_)
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
