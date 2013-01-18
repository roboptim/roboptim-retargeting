#include "roboptim/retargeting/acceleration-energy.hh"

namespace roboptim
{
  namespace retargeting
  {
    AccelerationEnergy::AccelerationEnergy
    (InteractionMeshShPtr_t mesh) throw ()
      : roboptim::Function
	(boost::num_vertices (mesh->graph ()) * 3 * 1,
	 1, ""),
	mesh_ (mesh)
    {}

    AccelerationEnergy::~AccelerationEnergy () throw ()
    {}

    void
    AccelerationEnergy::impl_compute
    (result_t& result, const argument_t&)
      const throw ()
    {
      //FIXME: one frame for now, so 0.
      result.resize (1);
      result[0] = 0.;
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
