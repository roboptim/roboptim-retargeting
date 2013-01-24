#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/torque.hh"

namespace roboptim
{
  namespace retargeting
  {
    Torque::Torque
    () throw ()
      : roboptim::DifferentiableFunction
	(1,
	 1, "")
    {}

    Torque::~Torque () throw ()
    {}

    void
    Torque::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
    }

    void
    Torque::impl_gradient
    (gradient_t& gradient,
     const argument_t& x,
     size_type i)
      const throw ()
    {
      roboptim::FiniteDifferenceGradient<>
	fdg (*this);
      fdg.gradient (gradient, x, i);
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
