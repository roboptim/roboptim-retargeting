#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/position.hh"

namespace roboptim
{
  namespace retargeting
  {
    Position::Position
    () throw ()
      : roboptim::DifferentiableFunction
	(1,
	 1, "")
    {}

    Position::~Position () throw ()
    {}

    void
    Position::impl_compute
    (result_t&, const argument_t&)
      const throw ()
    {
    }

    void
    Position::impl_gradient
    (gradient_t& gradient,
     const argument_t& x,
     size_type i)
      const throw ()
    {
      roboptim::FiniteDifferenceGradient<
	finiteDifferenceGradientPolicies::Simple>
	fdg (*this);
      fdg.gradient (gradient, x, i);
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
