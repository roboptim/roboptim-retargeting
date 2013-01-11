#ifndef ROBOPTIM_RETARGETING_DEFORMATION_ENERGY_HH
# define ROBOPTIM_RETARGETING_DEFORMATION_ENERGY_HH
# include <roboptim/core/function.hh>

namespace roboptim
{
  class LaplacianDeformationEnergy : public roboptim::Function
  {
  public:
    explicit LaplacianDeformationEnergy ();
    virtual ~LaplacianDeformationEnergy () throw ();
    void impl_compute (result_t& result, const argument_t& x)
      const throw ();
  private:
  };
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_DEFORMATION_ENERGY_HH
