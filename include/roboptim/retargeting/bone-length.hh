#ifndef ROBOPTIM_RETARGETING_BONE_LENGTH_HH
# define ROBOPTIM_RETARGETING_BONE_LENGTH_HH
# include <boost/shared_ptr.hpp>
# include <roboptim/core/numeric-linear-function.hh>
# include <roboptim/retargeting/bone.hh>

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
      public roboptim::GenericNumericLinearFunction<
      roboptim::EigenMatrixSparse>
    {
    public:
      explicit BoneLength
      (cnoid::MarkerIMeshPtr markerIMesh,
       boost::shared_ptr<std::vector<CharacterInfo> > characterInfos,
       int numAllBones) throw ();
      virtual ~BoneLength () throw ();

      // void impl_compute (result_t& result, const argument_t& x)
      // 	const throw ();
      // void impl_gradient (gradient_t& gradient,
      // 			  const argument_t& argument,
      // 			  size_type functionId = 0)
      // 	const throw ();
      // void impl_jacobian (jacobian_t& jacobian, const argument_t& arg)
      // 	const throw ();

    private:
      void initVariables();
      void extractBones();
      void setInteractionMesh(cnoid::MarkerIMeshPtr mesh);
      void setBoneLengthMatrixAndVectorOfFrame(double alpha);
      void initFrame(int frame);
      void copySolution();

      boost::shared_ptr<std::vector<CharacterInfo> > characterInfos;

      cnoid::MarkerIMeshPtr mesh;

      std::vector<cnoid::MarkerMotionPtr> morphedMarkerMotions;

      std::vector<cnoid::MarkerMotion::Frame> Vi0_frames; // original positions
      std::vector<cnoid::MarkerMotion::Frame> Vi_frames;  // current (morphed) positions

      // key: active vertex index of a vertex of an edge
      // value: global vertex index of the other vertex of the edge
      std::map<int, std::set<int> > boneEdgeMap;

      int currentFrame;
      cnoid::VectorXd x; // solution

      int m;  // the number of "active" vertices
      int m3; // the number of all the active vertex elements (m * 3)
      int m3n; // m3 * number of frames;
      int numAllBones; // the number of "active" bones
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_BONE_LENGTH_HH
