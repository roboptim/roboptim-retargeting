#ifndef ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_HH
# define ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_HH
# include <boost/format.hpp>
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Compute a particular body position for a given configuration.
    ///
    /// Input: [q0, q1, ..., qN]
    /// Output: [bodyM_x, bodyM_y, bodyM_z]
    template <typename T>
    class ForwardGeometry : public GenericDifferentiableFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericDifferentiableFunction<T>);

      explicit ForwardGeometry (size_type nDofs,
				std::string title)
	throw ()
	: GenericDifferentiableFunction<T>
	  (nDofs, 3, (boost::format ("ForwardGeometry [%s]") % title).str ())
      {}

      virtual ~ForwardGeometry () throw ()
      {}
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_HH
