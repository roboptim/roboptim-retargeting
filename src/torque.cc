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
    (gradient_t&,
     const argument_t&,
     size_type)
      const throw ()
    {
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
