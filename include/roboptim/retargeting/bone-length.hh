#ifndef ROBOPTIM_RETARGETING_BONE_LENGTH_HH
# define ROBOPTIM_RETARGETING_BONE_LENGTH_HH
# include <boost/shared_ptr.hpp>
# include <roboptim/core/numeric-linear-function.hh>
# include <roboptim/retargeting/bone.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

# include <cnoid/ext/MocapPlugin/MarkerIMesh.h>

namespace roboptim
{
  namespace retargeting
  {
    class BoneLength;
    typedef boost::shared_ptr<BoneLength> BoneLengthShPtr_t;

    /// \brief Bone length constraint.
    ///
    /// Compute segment length error w.r.t the goal robot model.
    ///
    /// \f[
    /// C_B(V_i') = B_i V_i - l
    /// \f]
    class BoneLength :
      public roboptim::GenericNumericLinearFunction<
      roboptim::EigenMatrixSparse>
    {
    public:
      explicit BoneLength
      (cnoid::MarkerIMeshPtr markerIMesh,
       boost::shared_ptr<std::vector<CharacterInfo> > characterInfos,
       int numAllBones) throw ();
      virtual ~BoneLength () throw ();
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_BONE_LENGTH_HH
