#ifndef ROBOPTIM_RETARGETING_ACCELERATION_ENERGY_HH
# define ROBOPTIM_RETARGETING_ACCELERATION_ENERGY_HH
# include <roboptim/core/differentiable-function.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

namespace roboptim
{
  namespace retargeting
  {
    class AccelerationEnergy;

    typedef boost::shared_ptr<AccelerationEnergy>
    AccelerationEnergyShPtr_t;

    /// \brief Compute acceleration based term that smooths result
    ///        trajectory.
    class AccelerationEnergy : public roboptim::DifferentiableFunction
    {
    public:
      explicit AccelerationEnergy
      (AnimatedInteractionMeshShPtr_t animatedMesh) throw ();

      virtual ~AccelerationEnergy () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();

    private:
      AnimatedInteractionMeshShPtr_t animatedMesh_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_ACCELERATION_ENERGY_HH
