#ifndef ROBOPTIM_RETARGETING_LAPLACIAN_COORDINATE_HH
# define ROBOPTIM_RETARGETING_LAPLACIAN_COORDINATE_HH
# include <log4cxx/logger.h>

# include <boost/shared_ptr.hpp>

# include <roboptim/core/linear-function.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

namespace roboptim
{
  namespace retargeting
  {
    class LaplacianCoordinate;
    typedef boost::shared_ptr<LaplacianCoordinate> LaplacianCoordinateShPtr_t;
    /// \brief Laplacian Coordinates
    ///
    /// Compute the Laplacian coordinates of a node from a particular
    /// interactive mesh.
    ///
    /// \f[
    /// L(p_j) = p_j - \sum_{l \in N_j} w_l^j p_l
    /// \f]
    class LaplacianCoordinate : public roboptim::LinearFunction
      {
      public:
	using roboptim::Function::size_type;

	explicit LaplacianCoordinate
	(AnimatedInteractionMeshShPtr_t mesh,
	 AnimatedInteractionMesh::vertex_descriptor_t edge,
	 unsigned frameId_) throw ();
	virtual ~LaplacianCoordinate () throw ();
	void impl_compute (result_t& result, const argument_t& x)
	  const throw ();
	void impl_gradient (gradient_t& gradient,
			    const argument_t& argument,
			    size_type functionId = 0)
	  const throw ();
      private:
	static log4cxx::LoggerPtr logger;
	AnimatedInteractionMeshShPtr_t mesh_;
	AnimatedInteractionMesh::vertex_descriptor_t vertex_;
	unsigned frameId_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_LAPLACIAN_COORDINATE_HH

