#ifndef ROBOPTIM_RETARGETING_ACCELERATION_ENERGY_HH
# define ROBOPTIM_RETARGETING_ACCELERATION_ENERGY_HH
# include <boost/shared_ptr.hpp>
# include <roboptim/core/numeric-quadratic-function.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

# include <cnoid/ext/MocapPlugin/MarkerIMesh.h>

namespace roboptim
{
  namespace retargeting
  {
    class AccelerationEnergy;

    typedef boost::shared_ptr<AccelerationEnergy>
    AccelerationEnergyShPtr_t;

    /// \brief Compute acceleration based term that smooths result
    ///        trajectory.
    ///
    /// \f[
    /// E_A(V_{i-1}',V_i',V_{i+1}') = \frac{1}{2} \| V_{i-1}^2 + 2 V_i + V_{i+1}^2 \|^2
    /// \f]

    class AccelerationEnergy
      : public roboptim::GenericNumericQuadraticFunction<EigenMatrixSparse>
    {
    public:
      explicit AccelerationEnergy
      (cnoid::MarkerIMeshPtr markerIMesh) throw ();

      virtual ~AccelerationEnergy () throw ();
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_ACCELERATION_ENERGY_HH
