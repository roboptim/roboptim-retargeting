#ifndef ROBOPTIM_RETARGETING_FUNCTION_ZMP_HH
# define ROBOPTIM_RETARGETING_FUNCTION_ZMP_HH
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    namespace zmpPolicies
    {
    /// \brief Compute the Torque of each joint.
    ///
    /// Input:
    ///  x = [q, \dot{q}, \ddot{q}]
    ///
    /// Output:
    ///  result = [\tau_0, \tau_1, ..., \tau_N]
    ///
    /// \tparam T Function traits type
    template <typename T>
    class ZMP : public T
    {
    public:
      explicit ZMP (std::string title,
		    size_type nDofs) throw ()
	: T (3, nDofs, std::string ("Torque [") + title + "]")
      {}

      virtual ~ZMP () throw ()
      {}
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_ZMP_HH
