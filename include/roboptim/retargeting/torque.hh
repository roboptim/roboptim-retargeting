#ifndef ROBOPTIM_RETARGETING_TORQUE_HH
# define ROBOPTIM_RETARGETING_TORQUE_HH
# include <boost/shared_ptr.hpp>
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    class Torque;
    typedef boost::shared_ptr<Torque> TorqueShPtr_t;

    /// \brief Torque constraints (not yet implemented)
    class Torque : public roboptim::DifferentiableFunction
    {
    public:
      explicit Torque () throw ();
      virtual ~Torque () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();
    private:
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_TORQUE_HH
