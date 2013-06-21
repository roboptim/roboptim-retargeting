#ifndef ROBOPTIM_RETARGETING_ZMP_HH
# define ROBOPTIM_RETARGETING_ZMP_HH
# include <boost/shared_ptr.hpp>
# include <roboptim/core/differentiable-function.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

# include <cnoid/ext/MocapPlugin/MarkerMotion.h>
# include <cnoid/src/MocapPlugin/MarkerToBodyMotionConverter.h>

namespace roboptim
{
  namespace retargeting
  {
    class ZMP;
    typedef boost::shared_ptr<ZMP> ZmpShPtr_t;

    /// \brief ZMP constraints (not yet implemented)
    class ZMP :
      public roboptim::GenericDifferentiableFunction<EigenMatrixSparse>
    {
    public:
      explicit ZMP (AnimatedInteractionMeshShPtr_t animatedMesh,
		    AnimatedInteractionMeshShPtr_t animatedMeshLocal,
		    cnoid::BodyPtr model,
		    cnoid::MarkerMotionPtr mocapMotion)
	throw ();
      virtual ~ZMP () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();
    private:
      AnimatedInteractionMeshShPtr_t animatedMesh_;
      AnimatedInteractionMeshShPtr_t animatedMeshLocal_;

      cnoid::BodyPtr model_;
      mutable cnoid::MarkerMotionPtr mocapMotion_;
      mutable cnoid::MarkerToBodyMotionConverter converter_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_ZMP_HH
