#ifndef ROBOPTIM_RETARGETING_FUNCTION_ZMP_HH
# define ROBOPTIM_RETARGETING_FUNCTION_ZMP_HH
# include <string>

# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Abstract class for Computing the ZMP position.
    ///
    /// Input:
    ///  x = [q, \dot{q}, \ddot{q}]
    ///
    /// Output:
    ///  result = [zmp_x, zmp_y]
    ///
    /// Implementation must be done in zmp/*.hh depending on how the
    /// computation is done (metapod, choreonoid or linear model)
    ///
    /// \tparam T Function traits type
    template <typename T>
    class ZMP : public GenericDifferentiableFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericDifferentiableFunction<T>);

      explicit ZMP (size_type nDofs, std::string title) throw ()
	: GenericDifferentiableFunction<T>
	  (nDofs * 3, 2, std::string ("ZMP [") + title + "]")
      {}

      virtual ~ZMP () throw ()
      {}
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_ZMP_HH
