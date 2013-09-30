#ifndef ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_HH
# define ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_HH
# include <cnoid/Body>

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
	: GenericDifferentiableFunction<T>
	  ((boost::format ("ForwardGeometry [%s]") % title).str (),
	   nDofs, 3)
      {}

      virtual ~ForwardGeometry ()
      {}

    private:
      cnoid::BodyPtr robot_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_HH
