#ifndef ROBOPTIM_RETARGETING_DEFORMATION_ENERGY_HH
# define ROBOPTIM_RETARGETING_DEFORMATION_ENERGY_HH
# include <vector>
# include <roboptim/core/linear-function.hh>

# include <roboptim/retargeting/laplacian-coordinate.hh>
# include <roboptim/retargeting/animated-interaction-mesh.hh>

# include <cnoid/ext/MocapPlugin/MarkerIMesh.h>

namespace roboptim
{
  namespace retargeting
  {
    class LaplacianDeformationEnergy;

    typedef boost::shared_ptr<LaplacianDeformationEnergy>
    LaplacianDeformationEnergyShPtr_t;

    /// \brief Compute that Laplacian Deformation Energy.
    ///
    /// \f[
    /// E_L(V_{i}') = \sum_{j} \| \delta_{j} - L(p_{j}'^{i}) \|^{2}
    /// \f]
    class LaplacianDeformationEnergy
      : public roboptim::GenericLinearFunction<EigenMatrixSparse>
    {
    public:
      explicit LaplacianDeformationEnergy
      (AnimatedInteractionMeshShPtr_t mesh,
       cnoid::MarkerMotionPtr originalMarkerMotion,
       cnoid::CharacterPtr character) throw ();

      virtual ~LaplacianDeformationEnergy () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();
    private:
      /// \brief Original mesh.
      AnimatedInteractionMeshShPtr_t animatedMesh_;
      /// \brief Current state of the mesh (during optimization process).
      AnimatedInteractionMeshShPtr_t animatedMeshLocal_;

      mutable Eigen::VectorXd buffer_;

      mutable cnoid::MarkerMotionPtr markerMotion_;

      void initVariables ();
      void extractBones ();

      const cnoid::Vector3& orgVertexOfActiveIndex(int activeIndex) const
      {
	const cnoid::MarkerIMesh::LocalIndex& localIndex = mesh->activeToLocalIndex(activeIndex);
	return Vi0_frames[localIndex.motionIndex][localIndex.markerIndex];
      }

      const cnoid::Vector3& orgVertexOfGlobalIndex(int globalIndex) const
      {
	const cnoid::MarkerIMesh::LocalIndex& localIndex = mesh->globalToLocalIndex(globalIndex);
	return Vi0_frames[localIndex.motionIndex][localIndex.markerIndex];
      }
      
      mutable std::vector<cnoid::MarkerMotionPtr> morphedMarkerMotions;

      mutable cnoid::MarkerIMeshPtr mesh;

      std::vector<cnoid::MarkerMotion::Frame> Vi0_frames; // original positions
      std::vector<cnoid::MarkerMotion::Frame> Vi_frames;  // current (morphed) positions


      // key: active vertex index of a vertex of an edge
      // value: global vertex index of the other vertex of the edge
      mutable std::map<int, std::set<int> > boneEdgeMap;

      // For the deformation energy
      int m;
      int m3;
      int  numAllBones; // the number of "active" bones
      mutable std::vector<cnoid::MatrixXd> M;
      mutable std::vector<cnoid::VectorXd> b;
      mutable std::vector<cnoid::MatrixXd> MtM;
      double laplacianWeightPowerHalf;
      bool doUpdateLaplacianCoordinateConstantEveryFrame;


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

        bool doExcludeBoneEdgesFromLaplacianCoordinate;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_DEFORMATION_ENERGY_HH
