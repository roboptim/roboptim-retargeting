#ifndef ROBOPTIM_RETARGETING_LAPLACIAN_COORDINATE_HH
# define ROBOPTIM_RETARGETING_LAPLACIAN_COORDINATE_HH
# include <log4cxx/logger.h>
# include <roboptim/core/function.hh>

# include <roboptim/retargeting/interaction-mesh.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Laplacian Coordinates
    ///
    /// Compute the Laplacian coordinates of a node from a particular
    /// interactive mesh.
    ///
    /// \f[
    /// L(p_j) = p_j - \sum_{l \in N_j} w_l^j p_l
    /// \f]
    class LaplacianCoordinate : public roboptim::Function
      {
      public:
	using roboptim::Function::size_type;

	explicit LaplacianCoordinate
	(InteractionMeshShPtr_t mesh,
	 InteractionMesh::vertex_descriptor_t edge) throw ();
	virtual ~LaplacianCoordinate () throw ();
	void impl_compute (result_t& result, const argument_t& x)
	  const throw ();
      private:
	static log4cxx::LoggerPtr logger;
	InteractionMeshShPtr_t mesh_;
	InteractionMesh::vertex_descriptor_t vertex_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_LAPLACIAN_COORDINATE_HH

