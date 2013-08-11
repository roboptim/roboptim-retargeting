#ifndef ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_HH
# define ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_HH
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Compute forward geometry for a particular robot model.
    ///
    /// Robot model from choreonoid should be passed to the
    /// constructor and use for computation.  The analytical jacobian
    /// must also be retrieved and returned in the
    /// impl_gradient/impl_jacobian methods.
    template <typename T>
    class ForwardGeometry : public GenericDifferentiableFunction<T>
    {
    public:
      explicit ForwardGeometry ()
	: GenericDifferentiableFunction ("ForwardGeometry", 1, 1*3)
      {
      }

      ~ForwardGeometry ()
      {
      }
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_HH
