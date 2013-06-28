#ifndef ROBOPTIM_RETARGETING_POSITION_HH
# define ROBOPTIM_RETARGETING_POSITION_HH
# include <boost/shared_ptr.hpp>
# include <roboptim/core/linear-function.hh>

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
      public roboptim::GenericLinearFunction<EigenMatrixSparse>
    {
    public:
      explicit Position
      (cnoid::MarkerIMeshPtr markerIMesh,
       int motionIndex,
       int localMarkerIndex,
       const cnoid::Vector3& pos,
       bool isRelative,
       double alpha,
       double weight = 1000.0) throw ();
      virtual ~Position () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();
    private:
      int activeVertexIndex;
      cnoid::Vector3 pos;
      bool isRelative;
      //optional<Vector3> pos; // invalid when the original position is used
      double alpha;
      double weight; // only for soft constraint

      void initVariables();
      void extractBones();
      void setInteractionMesh(cnoid::MarkerIMeshPtr mesh);
      void initFrame(int frame) const;
      void copySolution() const;
      void setPositionalConstraintMatrixAndVectorsOfFrame
      (matrix_t& Ki, cnoid::VectorXd& Pi, int rowOffset, double alpha) const;



        const cnoid::Vector3& orgVertexOfActiveIndex(int activeIndex) const {
	  const cnoid::MarkerIMesh::LocalIndex& localIndex = mesh->activeToLocalIndex(activeIndex);
            return Vi0_frames[localIndex.motionIndex][localIndex.markerIndex];
        }
        const cnoid::Vector3& orgVertexOfGlobalIndex(int globalIndex) const {
            const cnoid::MarkerIMesh::LocalIndex& localIndex = mesh->globalToLocalIndex(globalIndex);
            return Vi0_frames[localIndex.motionIndex][localIndex.markerIndex];
        }

      mutable cnoid::MarkerIMeshPtr mesh;

      mutable std::vector<cnoid::MarkerMotionPtr> morphedMarkerMotions;

      mutable std::vector<cnoid::MarkerMotion::Frame> Vi0_frames; // original positions
      mutable std::vector<cnoid::MarkerMotion::Frame> Vi_frames;  // current (morphed) positions

      // key: active vertex index of a vertex of an edge
      // value: global vertex index of the other vertex of the edge
      mutable std::map<int, std::set<int> > boneEdgeMap;

      mutable cnoid::VectorXd hi;
      mutable std::vector<matrix_t> H;

      mutable int currentFrame;
      mutable cnoid::VectorXd x; // solution

      mutable int m;  // the number of "active" vertices
      mutable int m3; // the number of all the active vertex elements (m * 3)
      mutable int m3n; // m3 * number of frames;
      mutable int numAllBones; // the number of "active" bones
      mutable bool firstIter;
      bool isSingleFrameMode;

    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_POSITION_HH
