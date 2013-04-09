#ifndef ROBOPTIM_RETARGETING_COLLISION_HH
# define ROBOPTIM_RETARGETING_COLLISION_HH
# include <boost/shared_ptr.hpp>
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    class Collision;
    typedef boost::shared_ptr<Collision> CollisionShPtr_t;

    /// \brief Collision constraint (not implemented).
    class Collision :
      public roboptim::GenericDifferentiableFunction<EigenMatrixSparse>
    {
    public:
      explicit Collision () throw ();
      virtual ~Collision () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();
    private:
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_COLLISION_HH
