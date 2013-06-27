#ifndef ROBOPTIM_RETARGETING_TORQUE_HH
# define ROBOPTIM_RETARGETING_TORQUE_HH
# include <boost/shared_ptr.hpp>
# include <cnoid/Body>
# include <roboptim/core/differentiable-function.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

# include <cnoid/ext/MocapPlugin/MarkerMotion.h>
# include <cnoid/src/MocapPlugin/MarkerToBodyMotionConverter.h>

# include <cnoid/BodyMotion>

#include "model/hrp4g2.hh"

// Define which robot to use.
typedef metapod::hrp4g2 robot_t;


namespace roboptim
{
  namespace retargeting
  {
    class Torque;
    typedef boost::shared_ptr<Torque> TorqueShPtr_t;

    /// \brief Torque constraints (not yet implemented)
    class Torque :
      public roboptim::GenericDifferentiableFunction<EigenMatrixSparse>
    {
    public:
      explicit Torque (AnimatedInteractionMeshShPtr_t animatedMesh,
		       AnimatedInteractionMeshShPtr_t animatedMeshLocal,
		       cnoid::BodyPtr model,
		       cnoid::MarkerMotionPtr mocapMotion)
	throw ();
      virtual ~Torque () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();
    private:
      AnimatedInteractionMeshShPtr_t animatedMesh_;
      AnimatedInteractionMeshShPtr_t animatedMeshLocal_;
      std::vector<unsigned> consideredDofs_;
      Function::intervals_t torqueLimits_;
      cnoid::BodyPtr model_;
      mutable cnoid::MarkerMotionPtr mocapMotion_;
      mutable cnoid::MarkerToBodyMotionConverter converter_;

      mutable std::vector<robot_t::confVector> q;
      mutable std::vector<robot_t::confVector> dq;
      mutable std::vector<robot_t::confVector> ddq;
      mutable cnoid::BodyMotion robotMotion;
      mutable robot_t robot;
      mutable robot_t::confVector torques;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_TORQUE_HH
