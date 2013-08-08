#ifndef ROBOPTIM_RETARGETING_FUNCTION_ZMP_HH
# define ROBOPTIM_RETARGETING_FUNCTION_ZMP_HH
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    namespace zmpPolicies
    {
    /// \brief Compute the ZMP position.
    ///
    /// Input:
    ///  x = [q, \dot{q}, \ddot{q}]
    ///
    /// Output:
    ///  result = [zmp_x, zmp_y, 0]
    ///
    /// \tparam T Function traits type
    template <typename T>
    class ZMP : public T
    {
    public:
      explicit ZMP (std::string title) throw ()
	: T (3, 2, std::string ("ZMP [") + title + "]")
      {}

      virtual ~ZMP () throw ()
      {}
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_ZMP_HH
