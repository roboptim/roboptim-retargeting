#ifndef ROBOPTIM_RETARGETING_POSITION_HH
# define ROBOPTIM_RETARGETING_POSITION_HH
# include <roboptim/core/function.hh>

namespace roboptim
{
  class Position : public roboptim::Function
  {
  public:
    explicit Position ();
    virtual ~Position () throw ();
    void impl_compute (result_t& result, const argument_t& x)
      const throw ();
  private:
  };
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_POSITION_HH
