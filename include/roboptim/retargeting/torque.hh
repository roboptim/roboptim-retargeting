#ifndef ROBOPTIM_RETARGETING_TORQUE_HH
# define ROBOPTIM_RETARGETING_TORQUE_HH
# include <boost/shared_ptr.hpp>
# include <urdf_interface/model.h>
# include <roboptim/core/differentiable-function.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

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
      explicit Torque (boost::shared_ptr<urdf::ModelInterface> model,
		       AnimatedInteractionMeshShPtr_t animatedMesh,
		       AnimatedInteractionMeshShPtr_t animatedMeshLocal)
	throw ();
      virtual ~Torque () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();
    private:
      boost::shared_ptr<urdf::ModelInterface> model_;
      AnimatedInteractionMeshShPtr_t animatedMesh_;
      AnimatedInteractionMeshShPtr_t animatedMeshLocal_;
      std::vector<unsigned> consideredDofs_;
      Function::intervals_t torqueLimits_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_TORQUE_HH
