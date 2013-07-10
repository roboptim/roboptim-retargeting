#include <fstream>
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

	      if (pe1ActiveIndex < 0 || pe2ActiveIndex < 0)
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
	    }
	}

      // Initialize matrices.
      A ().reserve (6 * numAllBones * mesh->numFrames ());
      for (int frameId = 0; frameId < mesh->numFrames (); ++frameId)
	{
	  int globalBoneId = 0;
	  int lineFrameOffset = frameId * numAllBones;
	  int columnFrameOffset = frameId * mesh->numActiveVertices () * 3;

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

	      std::cout << "num bones: " << chara.bones.size () << "\n";
	      for(std::size_t boneId = 0;
		  boneId < chara.bones.size (); ++boneId)
		{
		  std::cout << "bone id: " << boneId << "\n";
		  const Bone& bone = chara.bones[boneId];
		  if (!bone.goalLength)
		    throw std::runtime_error ("missing bone goal length");
		  cnoid::MarkerMotionPtr motion = mesh->motion (motionId_);

		  const cnoid::Vector3& boneStartPositionOriginal =
		    motion->frame(frameId)[bone.localMarkerIndex1];
		  const cnoid::Vector3&  boneEndPositionOriginal =
		    motion->frame(frameId)[bone.localMarkerIndex2];

		  cnoid::Vector3 delta =
		    boneStartPositionOriginal - boneEndPositionOriginal;
		  double norm = delta.norm ();
		  cnoid::Vector3 normalizedDelta =
		    delta / norm;

		  std::cout << delta << std::endl;
		  std::cout << "start: " << boneStartPositionOriginal << std::endl;
		  std::cout << "end: " << boneEndPositionOriginal << std::endl;

		  const int& offsetBoneStart = bone.activeVertexIndex1 * 3;
		  const int& offsetBoneEnd = bone.activeVertexIndex2 * 3;

		  // fill the vector part
		  this->b () (lineFrameOffset + globalBoneId) =
		    (*bone.goalLength - norm);
		  for (unsigned i = 0; i < 3; ++i)
		    {
		      this->b () (lineFrameOffset + globalBoneId) +=
			boneStartPositionOriginal[i] * normalizedDelta[i];
		      this->b () (lineFrameOffset + globalBoneId) -=
			boneEndPositionOriginal[i] * normalizedDelta[i];
		    }

		  this->b () (lineFrameOffset + globalBoneId) *= -1.;

		  // fill the jacobian
		  // - derivation w.r.t bone start
		  for (unsigned i = 0; i < 3; ++i)
		    A ().insert
		      (lineFrameOffset + globalBoneId,
		       columnFrameOffset + offsetBoneStart + i) =
		      normalizedDelta[i];

		  // - derivation w.r.t bone end
		  for (unsigned i = 0; i < 3; ++i)
		    A ().insert
		      (lineFrameOffset + globalBoneId,
		       columnFrameOffset + offsetBoneEnd + i) =
		      -normalizedDelta[i];

		  ++globalBoneId;
		}
	    }
	}
      A ().finalize ();

      // Check A structure.
      for (matrix_t::Index row = 0; row < A ().rows (); ++row)
	{
	  int nonZero = 0;
	  for (matrix_t::Index col = 0; col < A ().cols (); ++col)
	    if (A ().coeff (row, col) != 0.)
	      ++nonZero;
	  assert (nonZero == 2 * 3);
	}

      // Print A in file (dense representation).
      {
	std::ofstream file ("/tmp/bone-length-A.dat");
	file.setf (std::ios::right);
	for (Eigen::MatrixXd::Index row = 0; row < A ().rows (); ++row)
	  {
	    for (Eigen::MatrixXd::Index col = 0; col < A ().cols (); ++col)
	      file << std::showpos << std::fixed << std::setprecision (8)
		   << A ().coeff (row, col) << " ";
	    file << "\n";
	  }
      }
      {
	std::ofstream file ("/tmp/bone-length-b.dat");
	for (Eigen::MatrixXd::Index elt = 0; elt < b ().size (); ++elt)
	  file << b ()[elt] << "\n";
      }

      std::cout << "numallbones: " << numAllBones << std::endl;
    }

    BoneLength::~BoneLength () throw ()
    {}
  } // end of namespace retargeting.
} // end of namespace roboptim.
