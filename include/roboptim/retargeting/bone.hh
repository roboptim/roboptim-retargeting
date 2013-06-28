#ifndef ROBOPTIM_RETARGETING_BONE_HH
# define ROBOPTIM_RETARGETING_BONE_HH
# include <vector>
# include <boost/optional.hpp>
# include <cnoid/ext/MocapPlugin/Character.h>

namespace roboptim
{
  namespace retargeting
  {
    /**
       Index1 should be smaller than index2 to sequentially insert the
       coefficients of the bone length constraints into the sparse
       matrix.
    */
    struct Bone
    {
      int localMarkerIndex1;
      int localMarkerIndex2;
      int activeVertexIndex1;
      int activeVertexIndex2;
      boost::optional<double> goalLength;
    };

    struct CharacterInfo
    {
      cnoid::CharacterPtr org;
      cnoid::CharacterPtr goal;
      std::vector<Bone> bones;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_BONE_HH
