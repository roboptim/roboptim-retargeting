#include "roboptim/retargeting/sum.hh"

namespace roboptim
{
  namespace retargeting
  {
    Sum::Sum
    (const std::vector<DifferentiableFunctionShPtr_t>& functions) throw ()
      : roboptim::DifferentiableFunction
	(functions[0]->inputSize (),
	 functions[0]->outputSize (), ""),
	functions_ (functions)
    {}

    Sum::~Sum () throw ()
    {}

    void
    Sum::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      for (unsigned i = 0; i < functions_.size (); ++i)
	result += (*functions_[i]) (x);
    }

    void
    Sum::impl_gradient (gradient_t& gradient,
			const argument_t& argument,
			size_type functionId)
      const throw ()
    {
      for (unsigned i = 0; i < functions_.size (); ++i)
	gradient += functions_[i]->gradient (argument, functionId);
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.
