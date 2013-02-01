#ifndef ROBOPTIM_RETARGET_SUM_HH
# define ROBOPTIM_RETARGET_SUM_HH
# include <vector>
# include <boost/shared_ptr.hpp>

# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    class Sum;
    typedef boost::shared_ptr<roboptim::DifferentiableFunction>
    DifferentiableFunctionShPtr_t;
    typedef boost::shared_ptr<Sum> SumShPtr_t;

    class Sum : public roboptim::DifferentiableFunction
    {
    public:
      explicit Sum
      (const std::vector<DifferentiableFunctionShPtr_t>& functions) throw ();
      ~Sum () throw ();

      const std::vector<DifferentiableFunctionShPtr_t>& functions () const
      {
	return functions_;
      }

      std::vector<DifferentiableFunctionShPtr_t>& functions ()
      {
	return functions_;
      }

      void impl_compute (result_t& result, const argument_t& x)
	const throw ();

      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();
      void impl_jacobian (jacobian_t& jacobian,
			  const argument_t& arg)
	const throw ();
    private:
      std::vector<DifferentiableFunctionShPtr_t> functions_;
      mutable result_t result_;
      mutable gradient_t gradient_;
      mutable jacobian_t jacobian_;
    };

  }  // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGET_SUM_HH
