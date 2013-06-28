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
     double weight) throw ()
      : roboptim::GenericLinearFunction<EigenMatrixSparse>
	(markerIMesh->numFrames() * markerIMesh->numActiveVertices() * 3,
	 markerIMesh->numFrames() * 3,
	 "position"),
	activeVertexIndex
	(markerIMesh->localToActiveIndex(motionIndex, localMarkerIndex)),
	pos (pos),
	isRelative (isRelative),
	alpha (alpha),
	weight (weight),
	mesh (markerIMesh)
    {
      isSingleFrameMode = false;

      setInteractionMesh (markerIMesh);

      //extractBones();
      initVariables();

      firstIter = true;

      std::cout << "POSITION:" << pos << std::endl;
    }

    Position::~Position () throw ()
    {}

void Position::setInteractionMesh(cnoid::MarkerIMeshPtr mesh)
{
    this->mesh = mesh;
    m = mesh->numActiveVertices();
    m3 = m * 3;
    //constraintInfoSeq.resize(isSingleFrameMode ? 1 : mesh->numFrames());

    morphedMarkerMotions.resize(mesh->numMotions());
}


void Position::initVariables()
{
  bool isSingleFrameMode = false;
    int n = isSingleFrameMode ? 1 : mesh->numFrames();
    int m3n = m3 * n;

    H.resize(n);

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
        if(isSingleFrameMode){
            numMorphedFrames = 1;
            //orgFrameBegin = targetSingleFrame;
        } else if(mesh->numLocalActiveVertices(i) > 0){
            numMorphedFrames = mesh->numFrames();
        } else {
            numMorphedFrames = org->numFrames();
        }
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
    (matrix_t& Ki, cnoid::VectorXd& Pi, int rowOffset, double alpha) const
    {
      int row = rowOffset;
      int col = activeVertexIndex * 3;
      for(int k=0; k < 3; ++k){
	Ki.startVec(row + k);
	Ki.insertBack(row + k, col + k) = 1.0;
      }
      if(isRelative){
	Pi.segment<3>(row) = orgVertexOfActiveIndex(activeVertexIndex) + pos;
      } else {
	Pi.segment<3>(row) = alpha * pos + (1.0 - alpha) * orgVertexOfActiveIndex (activeVertexIndex);
      }
    }



void Position::copySolution() const
{
    int numAllFrames = isSingleFrameMode ? 1 : mesh->numFrames();
    for(size_t i=0; i < morphedMarkerMotions.size(); ++i){
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
                if(activeIndex >= 0){
                    morphedFrame[k] = x.segment<3>(frameOffset + activeIndex * 3);
                } else {
                    morphedFrame[k] = orgFrame[k];
                }
            }
        }
    }
}


void Position::initFrame(int frame) const
{
    Vi0_frames.clear();
    Vi_frames.clear();
    const int morphedFrameIndex = isSingleFrameMode ? 0 : frame;

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


    void
    Position::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      this->x = x;

      int numFrames = mesh->numFrames ();
      int targetFrame = 0; //targetSingleFrame

      for(currentFrame=0; currentFrame < numFrames; ++currentFrame){

	if(isSingleFrameMode){
	  if(firstIter){
	    initFrame(targetFrame);
	  }
	} else {
	  targetFrame = currentFrame;
	  initFrame(targetFrame);
	}

	matrix_t& Hi = H[currentFrame];
	Hi.resize(3, m3); // resize() makes Hi zero, too.
	hi.resize(3);

	setPositionalConstraintMatrixAndVectorsOfFrame
	  (Hi, hi, 0, alpha);

	Hi.finalize ();

	result.segment (currentFrame * 3, 3) =
	  Hi * x.segment (currentFrame * m3, m3) - hi;
      }

      copySolution ();
      firstIter = false;
      std::cout << "POSITION RESULT: " << result  << std::endl;
    }

    void
    Position::impl_gradient
    (gradient_t& gradient,
     const argument_t&,
     size_type i)
      const throw ()
    {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

      roboptim::GenericFiniteDifferenceGradient<
	EigenMatrixSparse,
	finiteDifferenceGradientPolicies::Simple<EigenMatrixSparse> >
	fdg (*this);

      fdg.gradient (gradient, x, i);
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
