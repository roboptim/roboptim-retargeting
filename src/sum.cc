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
	functions_ (functions),
	result_ (functions[0]->outputSize ()),
	gradient_ (functions[0]->inputSize ()),
	jacobian_ (functions[0]->inputSize (),
		   functions[0]->outputSize ())
    {}

    Sum::~Sum () throw ()
    {}

    void
    Sum::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      for (unsigned i = 0; i < functions_.size (); ++i)
	{
	  (*functions_[i]) (result_, x);
	  result += result_;
	}
    }

    void
    Sum::impl_gradient (gradient_t& gradient,
			const argument_t& argument,
			size_type functionId)
      const throw ()
    {
      for (unsigned i = 0; i < functions_.size (); ++i)
	{
	  functions_[i]->gradient (gradient_, argument, functionId);
	  gradient += gradient_;
	}
    }

    void
    Sum::impl_jacobian (jacobian_t& jacobian,
			const argument_t& argument)
      const throw ()
    {
      for (unsigned i = 0; i < functions_.size (); ++i)
	{
	  functions_[i]->jacobian (jacobian, argument);
	  jacobian += jacobian_;
	}
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
