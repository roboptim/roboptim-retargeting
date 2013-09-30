#ifndef ROBOPTIM_RETARGETING_FUNCTION_LAPLACIAN_DEFORMATION_ENERGY_HH
# define ROBOPTIM_RETARGETING_FUNCTION_LAPLACIAN_DEFORMATION_ENERGY_HH
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Laplacian Deformation Energy
    ///
    /// Input:
    ///  x = [q]
    ///
    /// Output:
    ///  result = [cost]
    ///
    /// \tparam T Function traits type
    template <typename T>
    class LaplacianDeformationEnergy : public GenericDifferentiableFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericDifferentiableFunction<T>);

      explicit LaplacianDeformationEnergy (size_type nDofs) throw ()
	: GenericDifferentiableFunction<T>
	  (nDofs, 1, "LaplacianDeformationEnergy")
      {}

      virtual ~LaplacianDeformationEnergy () throw ()
      {}

    protected:

      void
      impl_compute
      (result_t& result, const argument_t& x)
	const throw ()
      {
	//FIXME:
      }

      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type i)
	const throw ()
      {
	//FIXME:
      }
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_LAPLACIAN_DEFORMATION_ENERGY_HH
