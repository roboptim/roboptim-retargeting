#include <boost/format.hpp>
#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/bone-length.hh"

namespace roboptim
{
  namespace retargeting
  {
    BoneLength::BoneLength
    (cnoid::MarkerIMeshPtr mesh,
     boost::shared_ptr<std::vector<CharacterInfo> > characterInfos,
     int numAllBones) throw ()
      : roboptim::GenericNumericLinearFunction<EigenMatrixSparse>
	(matrix_t
	 (numAllBones * mesh->numFrames (),
	  mesh->numActiveVertices () * mesh->numFrames () * 3),
	 vector_t
	 (numAllBones * mesh->numFrames ()))
    {
      if (!characterInfos)
	throw std::runtime_error ("character information are missing");
      if (!mesh)
	throw std::runtime_error ("interactive mesh is missing");

      // key: active vertex index of a vertex of an edge
      // value: global vertex index of the other vertex of the edge
      std::map<int, std::set<int> > boneEdgeMap;

      // Extract bones.
      for (std::size_t characterId = 0;
	   characterId < characterInfos->size(); ++characterId)
	{
	  int characterId_ = static_cast<int> (characterId);
	  CharacterInfo& chara = (*characterInfos)[characterId];
	  chara.bones.clear();
	  if(!chara.org)
	    continue;

	  const cnoid::MarkerMotionPtr& motion =
	    mesh->motion (characterId_);
	  const int vertexIndexOffset = mesh->globalVertexIndexOffset
	    (characterId_);
	  for(int boneId = 0; boneId < chara.org->numMarkerEdges (); ++boneId)
	    {
	      const cnoid::Character::Edge& orgEdge =
		chara.org->markerEdge (boneId);
	      int pe1LocalIndex = motion->markerIndex (orgEdge.label[0]);
	      int pe2LocalIndex = motion->markerIndex (orgEdge.label[1]);

	      if (pe1LocalIndex > pe2LocalIndex)
		std::swap (pe1LocalIndex, pe2LocalIndex); // sort

	      const int pe1GlobalIndex = vertexIndexOffset + pe1LocalIndex;
	      const int pe1ActiveIndex = mesh->globalToActiveIndex
		(pe1GlobalIndex);

	      const int pe2GlobalIndex = vertexIndexOffset + pe2LocalIndex;
	      const int pe2ActiveIndex =
		mesh->globalToActiveIndex(pe2GlobalIndex);

	      if (pe1ActiveIndex || pe2ActiveIndex == 0)
		continue;

	      Bone bone;
	      bone.localMarkerIndex1 = pe1LocalIndex;
	      bone.localMarkerIndex2 = pe2LocalIndex;
	      bone.activeVertexIndex1 = pe1ActiveIndex;
	      bone.activeVertexIndex2 = pe2ActiveIndex;

	      if(chara.goal)
		bone.goalLength =
		  chara.goal->markerEdge (boneId).length;
	      chara.bones.push_back (bone);
	      boneEdgeMap[pe1ActiveIndex].insert (pe2GlobalIndex);
	      boneEdgeMap[pe2ActiveIndex].insert (pe1GlobalIndex);
	    }
	}

      // Initialize matrices.
      A ().reserve (6 * numAllBones * mesh->numFrames ());
      int globalBoneId = 0;
      for (int frameId = 0; frameId < mesh->numFrames (); ++frameId)
	{
	  int frameOffset = frameId * numAllBones;

	  for(std::size_t motionId = 0;
	      motionId < characterInfos->size (); ++motionId)
	    {
	      int motionId_ = static_cast<int> (motionId);
	      assert (characterInfos);

	      cnoid::MarkerMotionPtr motion =
		mesh->motion (motionId_);
	      if (!motion)
		throw std::runtime_error ("motion is missing");

	      CharacterInfo& chara = (*characterInfos)[motionId];
	      if(!chara.org)
		continue;

	      for(std::size_t boneId = 0;
		  boneId < chara.bones.size (); ++boneId)
		{
		  const Bone& bone = chara.bones[boneId];
		  if (!bone.goalLength)
		    throw std::runtime_error ("missing bone goal length");
		  cnoid::MarkerMotionPtr motion = mesh->motion (motionId_);

		  const cnoid::Vector3& boneStartPositionOriginal =
		    motion->frame(frameId)[bone.localMarkerIndex1];
		  const cnoid::Vector3&  boneEndPositionOriginal =
		    motion->frame(frameId)[bone.localMarkerIndex2];

		  const cnoid::Vector3 delta =
		    (boneEndPositionOriginal
		     - boneStartPositionOriginal).rowwise ().norm ();

		  const int& offsetBoneStart = bone.activeVertexIndex1 * 3;
		  const int& offsetBoneEnd = bone.activeVertexIndex2 * 3;

		  // fill the vector part
		  this->b () (frameOffset + globalBoneId) =
		    -1. * (*bone.goalLength);

		  // fill the jacobian
		  // - derivation w.r.t bone start
		  A ().insert (frameOffset + globalBoneId, offsetBoneStart + 0) =
		    -1. * delta[0];
		  A ().insert (frameOffset + globalBoneId, offsetBoneStart + 1) =
		    -1. * delta[1];
		  A ().insert (frameOffset + globalBoneId, offsetBoneStart + 2) =
		    -1. * delta[2];

		  // - derivation w.r.t bone end
		  A ().insert (frameOffset + globalBoneId, offsetBoneEnd + 0) =
		    delta[0];
		  A ().insert (frameOffset + globalBoneId, offsetBoneEnd + 1) =
		    delta[1];
		  A ().insert (frameOffset + globalBoneId, offsetBoneEnd + 2) =
		    delta[2];

		  ++globalBoneId;
		}
	    }
	}
      A ().finalize ();
    }

    BoneLength::~BoneLength () throw ()
    {}
  } // end of namespace retargeting.
} // end of namespace roboptim.
