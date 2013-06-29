#include <boost/format.hpp>
#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/bone-length.hh"

namespace roboptim
{
  namespace retargeting
  {
    BoneLength::BoneLength
    (cnoid::MarkerIMeshPtr markerIMesh,
     boost::shared_ptr<std::vector<CharacterInfo> > characterInfosTmp,
     int numAllBones) throw ()
      : roboptim::GenericNumericLinearFunction<EigenMatrixSparse>
	(matrix_t
	 (numAllBones * markerIMesh->numFrames (),
	  markerIMesh->numActiveVertices () * markerIMesh->numFrames () * 3),
	 vector_t
	 (numAllBones * markerIMesh->numFrames ())),

	characterInfos (characterInfosTmp),
	mesh (markerIMesh),

	numAllBones (0)
    {
      setInteractionMesh (markerIMesh);

      extractBones();
      initVariables();

      // Initialize matrices.
      double alpha = 0.;

      int numFrames = mesh->numFrames ();
      int targetFrame = 0; //targetSingleFrame


      for(currentFrame=0; currentFrame < numFrames; ++currentFrame)
	{
	  targetFrame = currentFrame;
	  initFrame(targetFrame);
	  this->setBoneLengthMatrixAndVectorOfFrame (alpha);
	  copySolution();
	}

      A ().finalize ();
    }

    BoneLength::~BoneLength () throw ()
    {}

    void BoneLength::extractBones()
    {
      //std::cout <<"BL EXTRACT BONES" << std::endl;
      boneEdgeMap.clear();

      for(size_t i=0; i < characterInfos->size(); ++i){
	//std::cout <<"BL CHARA" << std::endl;
	CharacterInfo& chara = (*characterInfos)[i];
        chara.bones.clear();
        if(chara.org){
	  //std::cout <<"BL CHARA OK" << std::endl;
	  const cnoid::MarkerMotionPtr& motion = mesh->motion(i);
	  const int vertexIndexOffset = mesh->globalVertexIndexOffset(i);
	  const int numBones = chara.org->numMarkerEdges();
	  for(int j=0; j < numBones; ++j){
	    //std::cout <<"BL BONE" << std::endl;
	    const cnoid::Character::Edge& orgEdge = chara.org->markerEdge(j);
	    int pe1LocalIndex = motion->markerIndex(orgEdge.label[0]);
	    int pe2LocalIndex = motion->markerIndex(orgEdge.label[1]);

	    if(pe1LocalIndex > pe2LocalIndex){
	      std::swap(pe1LocalIndex, pe2LocalIndex); // sort
	    }
	    const int pe1GlobalIndex = vertexIndexOffset + pe1LocalIndex;
	    const int pe1ActiveIndex = mesh->globalToActiveIndex(pe1GlobalIndex);
	    if(pe1ActiveIndex >= 0){
	      const int pe2GlobalIndex = vertexIndexOffset + pe2LocalIndex;
	      const int pe2ActiveIndex = mesh->globalToActiveIndex(pe2GlobalIndex);
	      if(pe2ActiveIndex >= 0){
		Bone bone;
		bone.localMarkerIndex1 = pe1LocalIndex;
		bone.localMarkerIndex2 = pe2LocalIndex;
		bone.activeVertexIndex1 = pe1ActiveIndex;
		bone.activeVertexIndex2 = pe2ActiveIndex;

		if(chara.goal){
		  const cnoid::Character::Edge& goalEdge = chara.goal->markerEdge(j);
		  bone.goalLength = goalEdge.length;
		}
		chara.bones.push_back(bone);
		++numAllBones;

		if(/*doExcludeBoneEdgesFromLaplacianCoordinate*/true){
		  boneEdgeMap[pe1ActiveIndex].insert(pe2GlobalIndex);
		  boneEdgeMap[pe2ActiveIndex].insert(pe1GlobalIndex);
		}
	      }
	    }
	  }
        }
      }
    }


    void BoneLength::initVariables()
    {
      bool isSingleFrameMode = false;
      int n = isSingleFrameMode ? 1 : mesh->numFrames();
      int m3n = m3 * n;

      //H.resize(n);

      const int size = m3n;

      for(size_t i=0; i < morphedMarkerMotions.size(); ++i)
	{
	  cnoid::MarkerMotionPtr org = mesh->motion(i);
	  cnoid::MarkerMotionPtr& morphed = morphedMarkerMotions[i];
	  if(!morphed)
	    morphed.reset (new cnoid::MarkerMotion ());

	  morphed->copySeqProperties(*org);
	  
	  int numMorphedFrames;
	  int orgFrameBegin = 0;
	  if(isSingleFrameMode)
	    {
	      numMorphedFrames = 1;
	      //orgFrameBegin = targetSingleFrame;
	    } else if(mesh->numLocalActiveVertices(i) > 0){
	    numMorphedFrames = mesh->numFrames();
	  } else {
	    numMorphedFrames = org->numFrames();
	  }
	  morphed->setDimension(numMorphedFrames, org->numMarkers());
	  
	  for(int j = 0; j < numMorphedFrames; ++j)
	    {
	      int orgFrameIndex = std::min(j + orgFrameBegin, org->numFrames() - 1);
	      cnoid::MarkerMotion::Frame orgFrame = org->frame(orgFrameIndex);
	      cnoid::MarkerMotion::Frame morphedFrame = morphed->frame(j);
	      std::copy(orgFrame.begin(), orgFrame.end(), morphedFrame.begin());
	    }
	}
    }


    void BoneLength::setInteractionMesh(cnoid::MarkerIMeshPtr mesh)
    {
      this->mesh = mesh;
      m = mesh->numActiveVertices();
      m3 = m * 3;
      //constraintInfoSeq.resize(isSingleFrameMode ? 1 : mesh->numFrames());

      morphedMarkerMotions.resize(mesh->numMotions());
    }


    void
    BoneLength::setBoneLengthMatrixAndVectorOfFrame(double alpha)
    {
      int globalBoneIndex = 0;
      std::size_t frameOffset = currentFrame * numAllBones;

      for(size_t motionIndex=0; motionIndex < characterInfos->size(); ++motionIndex)
	{
	  CharacterInfo& chara = (*characterInfos)[motionIndex];
	  if(chara.org)
	    {
	      const cnoid::MarkerMotion::Frame& Vi0_frame = Vi0_frames[motionIndex];
	      const int numBones = chara.bones.size();
	      for(int j=0; j < numBones; ++j){
		const Bone& bone = chara.bones[j];
		cnoid::MarkerMotionPtr motion = mesh->motion(motionIndex);

		// Set desired bone length in this morph step
		const cnoid::Vector3& pe1_0 = Vi0_frame[bone.localMarkerIndex1];
		const cnoid::Vector3& pe2_0 = Vi0_frame[bone.localMarkerIndex2];
		double lgoal = (pe2_0 - pe1_0).norm(); // set original length first
		if(bone.goalLength){
		  lgoal = alpha * (*bone.goalLength) + (1.0 - alpha) * lgoal;
		}

		const cnoid::MarkerMotion::Frame& Vi_frame = Vi_frames[motionIndex];
		const cnoid::Vector3& pe1 = Vi_frame[bone.localMarkerIndex1];
		const cnoid::Vector3& pe2 = Vi_frame[bone.localMarkerIndex2];

		const double lcurrent = (pe1 - pe2).norm();

		double a = 0.;
		if(std::fabs(lcurrent) > 1.0e-6)
		  a = lgoal / lcurrent;
	     
		this->b () (frameOffset + globalBoneIndex) = (lgoal - lcurrent);

		cnoid::Vector3 dl;
		dl.setZero();
		if(fabs(lcurrent) > 1.0e-6)
		  dl = (1.0 / lcurrent) * (pe1 - pe2);

		this->A ().startVec(frameOffset + globalBoneIndex);

		int offset = bone.activeVertexIndex1 * 3;
		for(int j=0; j < 3; ++j)
		  {
		    this->A ().insertBack(frameOffset + globalBoneIndex, offset + j) = dl[j];
		    this->b () (frameOffset + globalBoneIndex) += dl[j] * pe1[j];
		  }

		offset = bone.activeVertexIndex2 * 3;
		for(int j=0; j < 3; ++j)
		  {
		    this->A ().insertBack (frameOffset + globalBoneIndex, offset + j) = -dl[j];
		    this->b () (frameOffset + globalBoneIndex) -= dl[j] * pe2[j];
		  }
		++globalBoneIndex;
	      }
	    }
	}
    }


    void BoneLength::copySolution()
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
	  for(int j=0; j < n; ++j)
	    {
	      cnoid::MarkerMotion::Frame morphedFrame = morphed->frame(j);
	      const int frameOffset = m3 * j;
	      const cnoid::MarkerMotion::Frame orgFrame = orgMotion->frame(std::min(j, orgMaxFrame));
	      for(int k=0; k < morphedFrame.size(); ++k)
		{
		  const int activeIndex = localToActiveIndexMap[k];
		  // if(activeIndex >= 0)
		  //   morphedFrame[k] = x.segment<3>(frameOffset + activeIndex * 3);
		  // else
		  morphedFrame[k] = orgFrame[k];
		}
	    }
	}
    }


    void BoneLength::initFrame(int frame)
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
