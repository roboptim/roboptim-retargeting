#ifndef ROBOPTIM_RETARGETING_FUNCTION_ZMP_INVERTED_PENDULUM_HH
# define ROBOPTIM_RETARGETING_FUNCTION_ZMP_INVERTED_PENDULUM_HH
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief ZMP position computed using the Inverted Pendulum
    ///        model.
    ///
    /// \tparam T Function traits type
    template <typename T>
    class ZMPInvertedPendulum : public ZMP<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_ (ZMP<T>);

      explicit ZMPInvertedPendulum (size_type nDofs) throw ()
	: ZMP<T> (nDofs, "choreonoid")
      {}

      virtual ~ZMPInvertedPendulum () throw ()
      {}

    protected:
      void
      impl_compute
      (result_t& result, const argument_t& x)
	const throw ()
      {
	//FIXME: implement this
      }

      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type i)
	const throw ()
      {
	//FIXME: implement this
      }
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_ZMP_INVERTED_PENDULUM_HH
