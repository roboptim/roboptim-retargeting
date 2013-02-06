#ifndef ROBOPTIM_RETARGETING_BONE_LENGTH_HH
# define ROBOPTIM_RETARGETING_BONE_LENGTH_HH
# include <boost/shared_ptr.hpp>
# include <roboptim/core/linear-function.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

namespace roboptim
{
  namespace retargeting
  {
    class BoneLength;
    typedef boost::shared_ptr<BoneLength> BoneLengthShPtr_t;

    class BoneLength : public roboptim::LinearFunction
    {
    public:
      explicit BoneLength
      (AnimatedInteractionMeshShPtr_t animatedMesh,
       AnimatedInteractionMeshShPtr_t animatedMeshLocal,
       AnimatedInteractionMesh::edge_descriptor_t edgeId) throw ();
      virtual ~BoneLength () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();
    private:
      AnimatedInteractionMeshShPtr_t animatedMesh_;
      AnimatedInteractionMeshShPtr_t animatedMeshLocal_;
      AnimatedInteractionMesh::edge_descriptor_t edgeId_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_BONE_LENGTH_HH
