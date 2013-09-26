#ifndef ROBOPTIM_RETARGETING_FUNCTION_TORQUE_METAPOD_HH
# define ROBOPTIM_RETARGETING_FUNCTION_TORQUE_METAPOD_HH
# include <boost/fusion/include/at_c.hpp>
# include <metapod/algos/rnea.hh>
# include <metapod/tools/print.hh>

# include <roboptim/core/finite-difference-gradient.hh>

# include <roboptim/retargeting/function/torque.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Torque position computed through Metapod
    ///
    /// \tparam T Function traits type
    /// \tparam R Robot type
    template <typename T, typename R>
    class TorqueMetapod : public Torque<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_ (Torque<T>);
      typedef R robot_t;

      explicit TorqueMetapod () throw ()
	: Torque<T> (robot_t::NBDOF, "metapod"),
	  robot_ ()
      {}

      virtual ~TorqueMetapod () throw ()
      {}

      // see https://github.com/laas/metapod/issues/63
      void
      impl_compute
      (result_t& result, const argument_t& x)
	const throw ()
      {
	metapod::rnea<robot_t, true>::run
	  (robot_,
	   x.segment (0 * robot_t::NBDOF, robot_t::NBDOF),
	   x.segment (1 * robot_t::NBDOF, robot_t::NBDOF),
	   x.segment (2 * robot_t::NBDOF, robot_t::NBDOF));
	metapod::getTorques (robot_, torques_);
	result = torques_;
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
      mutable robot_t robot_;
      mutable typename robot_t::confVector torques_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_TORQUE_HH
