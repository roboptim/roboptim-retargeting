#ifndef ROBOPTIM_RETARGET_SUM_HH
# define ROBOPTIM_RETARGET_SUM_HH
# include <vector>
# include <boost/shared_ptr.hpp>

# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Sum several RobOptim functions.
    ///
    /// \f[ sum_{(f_0, f_1, \cdots, f_i)}(x) = \sum_i f_i(x) \f]
    ///
    /// Associated jacobian:
    /// \f[ \frac{\partial sum_{(f_0, f_1, \cdots, f_i)}}{\partial x}(x) = \sum_i \frac{\partial f_i}{\partial x}(x) \f]
    template <typename T>
    class Sum : public roboptim::GenericDifferentiableFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (roboptim::GenericDifferentiableFunction<T>);

      typedef boost::shared_ptr<
      typename roboptim::GenericDifferentiableFunction<T> >
      DifferentiableFunctionShPtr_t;
      typedef boost::shared_ptr<Sum> SumShPtr_t;

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

# include <roboptim/retargeting/sum.hxx>
#endif //! ROBOPTIM_RETARGET_SUM_HH
