#ifndef ROBOPTIM_RETARGETING_FUNCTION_ZMP_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FUNCTION_ZMP_CHOREONOID_HH
# include <roboptim/core/finite-difference-gradient.hh>

# include <cnoid/Body>

# include <roboptim/retargeting/function/zmp.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief ZMP position computed through Choreonoid
    ///
    /// \tparam T Function traits type
    template <typename T>
    class ZMPChoreonoid : public ZMP<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_ (ZMP<T>);

      explicit ZMPChoreonoid (cnoid::BodyPtr robot) throw ()
	: ZMP<T> (robot->numJoints (), "choreonoid"),
	  robot_ (robot)
      {}

      virtual ~ZMPChoreonoid () throw ()
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
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	roboptim::GenericFiniteDifferenceGradient<
	  T,
	  finiteDifferenceGradientPolicies::Simple<T> >
	  fdg (*this);
	fdg.gradient (gradient, x, i);
      }

    private:
      cnoid::BodyPtr robot_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_ZMP_HH
