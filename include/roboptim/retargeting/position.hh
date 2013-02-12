#ifndef ROBOPTIM_RETARGETING_POSITION_HH
# define ROBOPTIM_RETARGETING_POSITION_HH
# include <boost/shared_ptr.hpp>
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    class Position;
    typedef boost::shared_ptr<Position> PositionShPtr_t;

    /// \brief Position constraint (not implemented)
    class Position : public roboptim::DifferentiableFunction
    {
    public:
      explicit Position () throw ();
      virtual ~Position () throw ();
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

#endif //! ROBOPTIM_RETARGETING_POSITION_HH
