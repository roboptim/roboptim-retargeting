#ifndef ROBOPTIM_RETARGETING_POSITION_HH
# define ROBOPTIM_RETARGETING_POSITION_HH
# include <boost/shared_ptr.hpp>
# include <roboptim/core/numeric-linear-function.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

# include <roboptim/retargeting/bone.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

# include <cnoid/ext/MocapPlugin/MarkerIMesh.h>

namespace roboptim
{
  namespace retargeting
  {
    class Position;
    typedef boost::shared_ptr<Position> PositionShPtr_t;

    /// \brief Position constraint (not implemented)
    class Position :
      public roboptim::GenericNumericLinearFunction<EigenMatrixSparse>
    {
    public:
      explicit Position
      (cnoid::MarkerIMeshPtr markerIMesh,
       int motionIndex,
       int localMarkerIndex,
       const cnoid::Vector3& pos,
       bool isRelative,
       double alpha,
       int numAllBones) throw ();
      virtual ~Position () throw ();

    private:
      void initVariables();
      void extractBones();
      void setInteractionMesh(cnoid::MarkerIMeshPtr mesh);
      void initFrame(int frame);
      void copySolution();
      void setPositionalConstraintMatrixAndVectorsOfFrame
      (int rowOffset, double alpha);

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

      boost::shared_ptr<matrix_t> H;
      boost::shared_ptr<vector_t> h;

      int activeVertexIndex;
      cnoid::Vector3 pos;
      bool isRelative;
      //optional<Vector3> pos; // invalid when the original position is used
      double alpha;

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
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_POSITION_HH
