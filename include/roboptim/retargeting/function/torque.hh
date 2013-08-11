#ifndef ROBOPTIM_RETARGETING_FUNCTION_TORQUE_HH
# define ROBOPTIM_RETARGETING_FUNCTION_TORQUE_HH
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
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
    class Torque : public T
    {
    public:
      explicit Torque (std::string title,
		    size_type nDofs) throw ()
	: T (3, nDofs, std::string ("Torque [") + title + "]")
      {}

      virtual ~Torque () throw ()
      {}
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_TORQUE_HH
