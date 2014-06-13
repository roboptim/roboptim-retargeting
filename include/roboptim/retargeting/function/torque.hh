#ifndef ROBOPTIM_RETARGETING_FUNCTION_TORQUE_HH
# define ROBOPTIM_RETARGETING_FUNCTION_TORQUE_HH
# include <string>

# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Compute the Torque of each joint.
    ///
    /// Input:
    /// \f$ x = [q, \dot{q}, \ddot{q}] \f$
    ///
    /// ...where q is [x, y, z, alpha, beta, gamma, q0, qN]
    ///
    /// (x, y, z) is the base position
    /// (alpha, beta, gamma) is base attitude using angle axis representation
    /// (q0, ..., qN) are the joints values
    ///
    /// Output:
    /// \f$ result = [\tau_0, \tau_1, ..., \tau_N] \f$
    ///
    /// \tparam T Function traits type
    template <typename T>
    class Torque : public GenericDifferentiableFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericDifferentiableFunction<T>);

      explicit Torque (size_type nDofs,
		       std::string title)
	: GenericDifferentiableFunction<T>
	  (nDofs * 3, nDofs, std::string ("Torque [") + title + "]")
      {}

      virtual ~Torque ()
      {}


      template <typename Derived>
      size_type
      configurationLength (const Eigen::MatrixBase<Derived>& x) const
      {
	return x.size () / 3;
      }

      template <typename Derived>
      Eigen::VectorBlock<const Derived>
      translation (const Eigen::MatrixBase<Derived>& x) const
      {
	return x.segment (0, 3);
      }

      template <typename Derived>
      Eigen::VectorBlock<Derived>
      translation (Eigen::MatrixBase<Derived>& x) const
      {
	return x.segment (0, 3);
      }

      template <typename Derived>
      Eigen::VectorBlock<const Derived>
      rotation (const Eigen::MatrixBase<Derived>& x) const
      {
	return x.segment (3, 3);
      }

      template <typename Derived>
      Eigen::VectorBlock<Derived>
      rotation (Eigen::MatrixBase<Derived>& x) const
      {
	return x.segment (3, 3);
      }

      template <typename Derived>
      Eigen::VectorBlock<const Derived>
      q (const Eigen::MatrixBase<Derived>& x,
	 bool withBaseLink = true) const
      {
	typename Eigen::VectorBlock<const Derived>::Index offset = 0;
	if (!withBaseLink)
	  offset += 6;
	return x.segment
	  (offset + 0 * configurationLength (x), configurationLength (x) - offset);
      }

      template <typename Derived>
      Eigen::VectorBlock<Derived>
      q (Eigen::MatrixBase<Derived>& x,
	 bool withBaseLink = true) const
      {
	typename Eigen::VectorBlock<Derived>::Index offset = 0;
	if (!withBaseLink)
	  offset += 6;
	return x.segment
	  (offset + 0 * configurationLength (x), configurationLength (x) - offset);
      }

      template <typename Derived>
      Eigen::VectorBlock<const Derived>
      dq (const Eigen::MatrixBase<Derived>& x,
	  bool withBaseLink = true) const
      {
	typename Eigen::VectorBlock<Derived>::Index offset = 0;
	if (!withBaseLink)
	  offset += 6;
	return x.segment
	  (offset + 1 * configurationLength (x), configurationLength (x) - offset);
      }

      template <typename Derived>
      Eigen::VectorBlock<Derived>
      dq (Eigen::MatrixBase<Derived>& x,
	  bool withBaseLink = true) const
      {
	std::size_t offset = 0;
	if (!withBaseLink)
	  offset += 6;
	return x.segment
	  (offset + 1 * configurationLength (x), configurationLength (x) - offset);
      }


      template <typename Derived>
      Eigen::VectorBlock<const Derived>
      ddq (const Eigen::MatrixBase<Derived>& x,
	   bool withBaseLink = true) const
      {
	typename Eigen::VectorBlock<Derived>::Index offset = 0;
	if (!withBaseLink)
	  offset += 6;
	return x.segment
	  (offset + 2 * configurationLength (x), configurationLength (x) - offset);
      }

      template <typename Derived>
      Eigen::VectorBlock<Derived>
      ddq (Eigen::MatrixBase<Derived>& x,
	   bool withBaseLink = true) const
      {
	std::size_t offset = 0;
	if (!withBaseLink)
	  offset += 6;
	return x.segment
	  (offset + 2 * configurationLength (x), configurationLength (x) - offset);
      }
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_TORQUE_HH
