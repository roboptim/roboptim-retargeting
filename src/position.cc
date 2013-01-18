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
    (result_t& result, const argument_t& x)
      const throw ()
    {
    }

    void
    Position::impl_gradient
    (gradient_t&,
     const argument_t&,
     size_type)
      const throw ()
    {
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
