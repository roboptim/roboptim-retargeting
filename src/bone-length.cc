#include "roboptim/retargeting/bone-length.hh"

namespace roboptim
{
  namespace retargeting
  {
    BoneLength::BoneLength
    () throw ()
      : roboptim::DifferentiableFunction
	(1,
	 1, "")
    {}

    BoneLength::~BoneLength () throw ()
    {}

    void
    BoneLength::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
    }

    void
    BoneLength::impl_gradient
    (gradient_t&,
     const argument_t&,
     size_type)
      const throw ()
    {
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
