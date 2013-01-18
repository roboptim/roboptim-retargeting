#ifndef ROBOPTIM_RETARGETING_RETARGET_HH
# define ROBOPTIM_RETARGETING_RETARGET_HH
# include <string>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

namespace roboptim
{
  namespace retargeting
  {
    class Retarget
    {
    public:
      explicit Retarget
      (const std::string& initialTrajectory);
      virtual ~Retarget ();
      void solve ();

      AnimatedInteractionMeshShPtr_t animatedMesh ()
      {
	return animatedMesh_;
      }

    private:
      AnimatedInteractionMeshShPtr_t animatedMesh_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_RETARGETING_HH
