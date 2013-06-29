#ifndef ROBOPTIM_RETARGET_SUM_HXX
# define ROBOPTIM_RETARGET_SUM_HXX

namespace roboptim
{
  namespace retargeting
  {
    template <typename T>
    std::string
    buildSumTitle
    (const std::vector<
      typename Sum<T>::DifferentiableFunctionShPtr_t>& functions)
    {
      std::string title ("∑");
      for (unsigned i = 0; i < functions.size (); ++i)
	title += " " + functions[i]->getName ();
      return title;
    }

    template <typename T>
    Sum<T>::Sum
    (const std::vector<DifferentiableFunctionShPtr_t>& functions) throw ()
      : roboptim::GenericDifferentiableFunction<T>
	(functions[0]->inputSize (),
	 functions[0]->outputSize (), buildSumTitle<T> (functions)),
	functions_ (functions),
	result_ (functions[0]->outputSize ()),
	gradient_ (functions[0]->inputSize ()),
	jacobian_ (functions[0]->outputSize (),
		   functions[0]->inputSize ())
    {
      result_.setZero ();
      gradient_.setZero ();
      jacobian_.setZero ();
    }

    template <typename T>
    Sum<T>::~Sum () throw ()
    {}

    template <typename T>
    void
    Sum<T>::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      result.setZero ();
      result_.setZero ();
      for (unsigned i = 0; i < functions_.size (); ++i)
	{
	  (*functions_[i]) (result_, x);
	  result += result_;
	}

      std::cout << "SUM" << result << std::endl;
    }

    template <typename T>
    void
    Sum<T>::impl_gradient (gradient_t& gradient,
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

    template <typename T>
    void
    Sum<T>::impl_jacobian (jacobian_t& jacobian,
			   const argument_t& argument)
      const throw ()
    {
      for (unsigned i = 0; i < functions_.size (); ++i)
	{
	  functions_[i]->jacobian (jacobian_, argument);
	  jacobian += jacobian_;
	}
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGET_SUM_HXX
