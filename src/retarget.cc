#include "roboptim/retargeting/retarget.hh"


namespace roboptim
{
  namespace retargeting
  {
    Retarget::Retarget (const std::string& initialTrajectory)
      : mesh_ (InteractionMesh::loadMesh (initialTrajectory))
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
