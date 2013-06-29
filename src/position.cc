#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/position.hh"

namespace roboptim
{
  namespace retargeting
  {
    Position::Position
    (cnoid::MarkerIMeshPtr markerIMesh,
     int motionIndex,
     int localMarkerIndex,
     const cnoid::Vector3& pos,
     bool isRelative,
     double alpha,
     int numAllBones) throw ()
      : roboptim::GenericNumericLinearFunction<EigenMatrixSparse>
	(matrix_t
	 (numAllBones * markerIMesh->numFrames (),
	  markerIMesh->numFrames() * markerIMesh->numActiveVertices() * 3),
	 vector_t
	 (numAllBones * markerIMesh->numFrames ())),

	activeVertexIndex
	(markerIMesh->localToActiveIndex (motionIndex, localMarkerIndex)),
	pos (pos),
	isRelative (isRelative),
	alpha (alpha),
	mesh (markerIMesh)
    {
      setInteractionMesh (markerIMesh);

      initVariables();

      int numFrames = mesh->numFrames ();
      int targetFrame = 0;

      for(currentFrame=0; currentFrame < numFrames; ++currentFrame)
	{
	  targetFrame = currentFrame;
	  initFrame(targetFrame);

	  setPositionalConstraintMatrixAndVectorsOfFrame
	    (0, alpha);

	  copySolution ();
	}

      this->A ().finalize ();
      std::cout << "POSITION:" << pos << std::endl;
    }

    Position::~Position () throw ()
    {}

    void Position::setInteractionMesh(cnoid::MarkerIMeshPtr mesh)
    {
      this->mesh = mesh;
      m = mesh->numActiveVertices();
      m3 = m * 3;
      morphedMarkerMotions.resize(mesh->numMotions());
    }


    void Position::initVariables()
    {
      int n = mesh->numFrames();
      int m3n = m3 * n;

      //H.resize(n);

      const int size = 3;

      for(size_t i=0; i < morphedMarkerMotions.size(); ++i){
        cnoid::MarkerMotionPtr org = mesh->motion(i);
        cnoid::MarkerMotionPtr& morphed = morphedMarkerMotions[i];
        if(!morphed){
	  morphed.reset(new cnoid::MarkerMotion());
        }
        morphed->copySeqProperties(*org);

        int numMorphedFrames;
        int orgFrameBegin = 0;
	if(mesh->numLocalActiveVertices(i) > 0)
	  numMorphedFrames = mesh->numFrames();
        else
	  numMorphedFrames = org->numFrames();

        morphed->setDimension(numMorphedFrames, org->numMarkers());

        for(int j = 0; j < numMorphedFrames; ++j){
	  int orgFrameIndex = std::min(j + orgFrameBegin, org->numFrames() - 1);
	  cnoid::MarkerMotion::Frame orgFrame = org->frame(orgFrameIndex);
	  cnoid::MarkerMotion::Frame morphedFrame = morphed->frame(j);
	  std::copy(orgFrame.begin(), orgFrame.end(), morphedFrame.begin());
        }
      }
    }


    void Position::setPositionalConstraintMatrixAndVectorsOfFrame
    (int rowOffset, double alpha)
    {
      int row = rowOffset;
      int col = activeVertexIndex * 3;
      for(int k=0; k < 3; ++k){
	this->A ().startVec(row + k);
	this->A ().insertBack(row + k, col + k) = 1.0;
      }
      if(isRelative){
	this->b ().segment<3>(row) = orgVertexOfActiveIndex(activeVertexIndex) + pos;
      } else {
	this->b ().segment<3>(row) = alpha * pos + (1.0 - alpha) * orgVertexOfActiveIndex (activeVertexIndex);
      }
    }



    void Position::copySolution()
    {
      int numAllFrames = mesh->numFrames();
      for(size_t i=0; i < morphedMarkerMotions.size(); ++i)
	{
	  cnoid::MarkerMotionPtr morphed = morphedMarkerMotions[i];
	  int index = 0;
	  const std::vector<int>& localToActiveIndexMap = mesh->localToActiveIndexMap(i);
	  cnoid::MarkerMotionPtr orgMotion = mesh->motion(i);
	  const int orgMaxFrame = orgMotion->numFrames() - 1;
	  const int n = std::min(numAllFrames, morphed->numFrames());
	  for(int j=0; j < n; ++j){
	    cnoid::MarkerMotion::Frame morphedFrame = morphed->frame(j);
	    const int frameOffset = m3 * j;
	    const cnoid::MarkerMotion::Frame orgFrame = orgMotion->frame(std::min(j, orgMaxFrame));
	    for(int k=0; k < morphedFrame.size(); ++k){
	      const int activeIndex = localToActiveIndexMap[k];
	      // if(activeIndex >= 0){
	      // 	morphedFrame[k] = x.segment<3>(frameOffset + activeIndex * 3);
	      // } else
	      morphedFrame[k] = orgFrame[k];
	    }
	  }
	}
    }


    void Position::initFrame(int frame)
    {
      Vi0_frames.clear();
      Vi_frames.clear();
      const int morphedFrameIndex = frame;

      for(int i=0; i < mesh->numMotions(); ++i){
        cnoid::MarkerMotionPtr motion = mesh->motion(i);
        const int maxFrame = motion->numFrames() - 1;
        cnoid::MarkerMotion::Frame orgFrame = motion->frame(std::min(frame, maxFrame));
        Vi0_frames.push_back(orgFrame);
        cnoid::MarkerMotion::Frame morphedFrame =
	  morphedMarkerMotions[i]->frame(std::min(morphedFrameIndex, maxFrame));
        Vi_frames.push_back(morphedFrame);
      }
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
