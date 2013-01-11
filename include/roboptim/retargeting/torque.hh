#ifndef ROBOPTIM_RETARGETING_COLLISION_HH
# define ROBOPTIM_RETARGETING_COLLISION_HH
# include <roboptim/core/function.hh>

namespace roboptim
{
  class Torque : public roboptim::Function
  {
  public:
    explicit Torque ();
    virtual ~Torque () throw ();
    void impl_compute (result_t& result, const argument_t& x)
      const throw ();
  private:
  };
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_COLLISION_HH
