#ifndef ROBOPTIM_RETARGETING_BONE_LENGTH_HH
# define ROBOPTIM_RETARGETING_BONE_LENGTH_HH
# include <boost/shared_ptr.hpp>
# include <roboptim/core/linear-function.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

# include <cnoid/ext/MocapPlugin/MarkerIMesh.h>

namespace roboptim
{
  namespace retargeting
  {
    class BoneLength;
    typedef boost::shared_ptr<BoneLength> BoneLengthShPtr_t;

    /// \brief Bone length constraint.
    ///
    /// Compute segment length error w.r.t the goal robot model.
    ///
    /// \f[
    /// C_B(V_i') = B_i V_i - l
    /// \f]
    class BoneLength :
      public roboptim::GenericLinearFunction<
      roboptim::EigenMatrixSparse>
    {
    public:
      explicit BoneLength
      (AnimatedInteractionMeshShPtr_t animatedMesh,
       AnimatedInteractionMeshShPtr_t animatedMeshLocal,
       AnimatedInteractionMesh::edge_descriptor_t edgeId) throw ();
      virtual ~BoneLength () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();
      void impl_jacobian (jacobian_t& jacobian, const argument_t& arg)
	const throw ();

    private:
      AnimatedInteractionMeshShPtr_t animatedMesh_;
      AnimatedInteractionMeshShPtr_t animatedMeshLocal_;
      AnimatedInteractionMesh::edge_descriptor_t edgeId_;
      AnimatedInteractionMesh::vertex_descriptor_t source_;
      AnimatedInteractionMesh::vertex_descriptor_t target_;
      /// \brief Scale corresponding to the selected edgeId.
      double scale_;
      /// \brief Segment goal length.
      /// I.e. alpha * |p1 - p2|
      double goalLength_;




        /**
           Index1 should be smaller than index2 to sequentially insert the coefficients
           of the bone length constraints into the sparse matrix.
        */
        struct Bone
        {
            int localMarkerIndex1;
            int localMarkerIndex2;
            int activeVertexIndex1;
            int activeVertexIndex2;
            boost::optional<double> goalLength;
        };

      struct CharacterInfo
      {
	cnoid::CharacterPtr org;
	cnoid::CharacterPtr goal;
	std::vector<Bone> bones;
      };
      std::vector<CharacterInfo> characterInfos;

      void setBoneLengthMatrixAndVectorOfFrame(matrix_t& Hi, double alpha);

      mutable cnoid::MarkerIMeshPtr mesh;

      mutable std::vector<cnoid::MarkerMotion::Frame> Vi0_frames; // original positions
      mutable std::vector<cnoid::MarkerMotion::Frame> Vi_frames;  // current (morphed) positions

      mutable cnoid::VectorXd hi;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_BONE_LENGTH_HH
