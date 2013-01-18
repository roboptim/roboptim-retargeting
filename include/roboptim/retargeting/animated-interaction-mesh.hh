#ifndef ROBOPTIM_RETARGETING_ANIMATED_INTERACTION_MESH
# define ROBOPTIM_RETARGETING_ANIMATED_INTERACTION_MESH
# include <vector>

# include <boost/shared_ptr.hpp>

# include <roboptim/retargeting/interaction-mesh.hh>

namespace roboptim
{
  namespace retargeting
  {
    class AnimatedInteractionMesh;
    typedef boost::shared_ptr<AnimatedInteractionMesh>
    AnimatedInteractionMeshShPtr_t;

    /// \brief Stores a set of interaction mesh representing a motion.
    class AnimatedInteractionMesh
    {
    public:
      explicit AnimatedInteractionMesh ();
      ~AnimatedInteractionMesh ();

      static AnimatedInteractionMeshShPtr_t loadAnimatedMesh
      (const std::string& trajectoryFile);


      const std::vector<InteractionMeshShPtr_t>& meshes () const
      {
	return meshes_;
      }

      std::vector<InteractionMeshShPtr_t>& meshes ()
      {
	return meshes_;
      }
    private:
      double framerate_;
      std::vector<InteractionMeshShPtr_t> meshes_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_ANIMATED_INTERACTION_MESH
