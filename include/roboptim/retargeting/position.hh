#ifndef ROBOPTIM_RETARGETING_POSITION_HH
# define ROBOPTIM_RETARGETING_POSITION_HH
# include <boost/shared_ptr.hpp>
# include <roboptim/core/linear-function.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

namespace roboptim
{
  namespace retargeting
  {
    class Position;
    typedef boost::shared_ptr<Position> PositionShPtr_t;

    /// \brief Position constraint (not implemented)
    class Position :
      public roboptim::GenericLinearFunction<EigenMatrixSparse>
    {
    public:
      explicit Position
      (AnimatedInteractionMeshShPtr_t animatedMesh,
       AnimatedInteractionMeshShPtr_t animatedMeshLocal,
       AnimatedInteractionMesh::vertex_descriptor_t vertexId,
       const Vertex::position_t& position) throw ();
      virtual ~Position () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();
    private:
      AnimatedInteractionMeshShPtr_t animatedMesh_;
      AnimatedInteractionMeshShPtr_t animatedMeshLocal_;
      AnimatedInteractionMesh::vertex_descriptor_t vertexId_;
      Vertex::position_t position_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_POSITION_HH
