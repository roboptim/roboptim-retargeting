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
	  (nDofs, 6, (boost::format ("ForwardGeometry [%s]") % title).str ())
      {}

      virtual ~ForwardGeometry () throw ()
      {}

      template <typename Derived>
      Eigen::VectorBlock<const Derived>
      translation (const Eigen::MatrixBase<Derived>& x) const throw ()
      {
	return x.segment (0, 3);
      }

      template <typename Derived>
      Eigen::VectorBlock<Derived>
      translation (Eigen::MatrixBase<Derived>& x) const throw ()
      {
	return x.segment (0, 3);
      }

      template <typename Derived>
      Eigen::VectorBlock<const Derived>
      rotation (const Eigen::MatrixBase<Derived>& x) const throw ()
      {
	return x.segment (3, 3);
      }

      template <typename Derived>
      Eigen::VectorBlock<Derived>
      rotation (Eigen::MatrixBase<Derived>& x) const throw ()
      {
	return x.segment (3, 3);
      }

      template <typename Derived>
      Eigen::VectorBlock<const Derived>
      q (const Eigen::MatrixBase<Derived>& x,
	 bool withBaseLink = true) const throw ()
      {
	std::size_t offset = 0;
	if (!withBaseLink)
	  offset += 6;
	return x.segment (offset, x.size () - offset);
      }

      template <typename Derived>
      Eigen::VectorBlock<Derived>
      q (Eigen::MatrixBase<Derived>& x,
	 bool withBaseLink = true) const throw ()
      {
	std::size_t offset = 0;
	if (!withBaseLink)
	  offset += 6;
	return x.segment (offset, x.size () - offset);
      }

    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_HH
