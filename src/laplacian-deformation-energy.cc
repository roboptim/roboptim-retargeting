#include <boost/make_shared.hpp>
#include <roboptim/core/finite-difference-gradient.hh>

#include "roboptim/retargeting/laplacian-deformation-energy.hh"
#include "roboptim/retargeting/laplacian-coordinate.hh"

// Remove trace logging in release.
#ifdef NDEBUG
# undef LOG4CXX_TRACE
# define LOG4CXX_TRACE(logger, msg)
#endif //!NDEBUG

namespace roboptim
{
  namespace retargeting
  {
    LaplacianDeformationEnergy::LaplacianDeformationEnergy
    (AnimatedInteractionMeshShPtr_t animatedMesh,
     cnoid::MarkerMotionPtr originalMarkerMotion,
     cnoid::CharacterPtr character) throw ()
      : roboptim::GenericLinearFunction<EigenMatrixSparse>
	(animatedMesh->optimizationVectorSize (),
	 1,
	 "laplacian deformation energy"),
	animatedMesh_ (animatedMesh),
	animatedMeshLocal_
	(AnimatedInteractionMesh::makeFromOptimizationVariables
	 (animatedMesh->state (), animatedMesh_)),
	buffer_ (3),
	markerMotion_ (originalMarkerMotion),
	mesh (new cnoid::MarkerIMesh ())
    {
      buffer_.setZero ();

      laplacianWeightPowerHalf = 1. / 2.;

      markerMotion_->copySeqProperties (*originalMarkerMotion);
      mesh->addMotion (originalMarkerMotion, character);
      mesh->update ();

      m = mesh->numActiveVertices ();
      m3 = m * 3;
      morphedMarkerMotions.resize (mesh->numMotions ());

      characterInfos.resize(1);
      CharacterInfo& chara = characterInfos[0];
      if(!chara.org){
	chara.org = character;
	chara.goal = character;
      }


      extractBones ();
      initVariables ();

      doExcludeBoneEdgesFromLaplacianCoordinate = true;

      for (int frame = 0; frame < markerMotion_->numFrames (); ++frame)
	for(int i=0; i < mesh->numMotions(); ++i)
	  {
	    cnoid::MarkerMotionPtr motion = mesh->motion(i);
	    const int maxFrame = motion->numFrames() - 1;
	    cnoid::MarkerMotion::Frame orgFrame =
	      motion->frame(std::min(frame, maxFrame));
	    Vi0_frames.push_back(orgFrame);
	    cnoid::MarkerMotion::Frame morphedFrame =
	      morphedMarkerMotions[i]->frame(std::min(frame, maxFrame));
	    Vi_frames.push_back(morphedFrame);
	  }



      for (int frameId = 0; frameId < m; ++frameId)
	{
	  cnoid::MatrixXd& Mi = M[frameId];
	  Mi.resize (m3, m3);
	  Mi.setZero ();
	  cnoid::VectorXd& bi = b[frameId];
	  bi.resize (m3);

	  std::vector<int> validNeighbors;
	  cnoid::MarkerIMesh::Frame neighborsList = mesh->frame (frameId);
	  for (int activeMarkerIndex = 0;
	       activeMarkerIndex < m; ++activeMarkerIndex)
	    {
	      const cnoid::MarkerIMesh::NeighborList& neighbors =
		neighborsList[activeMarkerIndex];
	      const std::set<int>& boneCounterparts =
		this->boneEdgeMap[activeMarkerIndex];
	      validNeighbors.clear ();

	      for (int k = 0; k < neighbors.size (); ++k)
		{
		  int l = neighbors[k];
		  if (boneCounterparts.find(l) == boneCounterparts.end())
		    {
		      // check if the edge is bone
		      validNeighbors.push_back(l);
		    }
		}

	      const cnoid::Vector3& pj =
		orgVertexOfActiveIndex (activeMarkerIndex);
	      const int j3 = activeMarkerIndex * 3;
	      Eigen::Block<cnoid::MatrixXd> a = Mi.block(j3, 0, 3, m3);
	      cnoid::VectorXd::FixedSegmentReturnType<3>::Type bij = bi.segment<3>(j3);
	      bij.setZero();
	      if(!validNeighbors.empty())
		{
		  double wn = 1.0 / validNeighbors.size ();
		  for (int k=0; k < validNeighbors.size (); ++k)
		    {
		      int l = validNeighbors[k];
		      const cnoid::Vector3& pl = orgVertexOfGlobalIndex(l);
		      const double w =
			wn * 1.0 / pow((pj - pl).squaredNorm(), laplacianWeightPowerHalf);
		      bij += w * pj;
		      const int l_active = mesh->globalToActiveIndex(l);
		      if(l_active >= 0)
			{
			  for(int s=0; s < 3; ++s){
			    a(s, l_active * 3 + s) = -w;
			  }
			  bij -= w * pl;
			}
		      for(int s=0; s < 3; ++s){
			a(s, j3 + s) +=  w;
			// Can .diagonal() be used here?
		      }
		    }
		}
	    }
	}
    }

    LaplacianDeformationEnergy::~LaplacianDeformationEnergy () throw ()
    {}

void LaplacianDeformationEnergy::initVariables()
{
  bool isSingleFrameMode = false;
    int n = isSingleFrameMode ? 1 : mesh->numFrames();
    int m3n = m3 * n;

    M.resize(n);
    b.resize(n);
    MtM.resize(n);
    //H.resize(n);

    // investigating the numbers of soft / hard constraints
    // int totalSoftConstraintSize = 0;
    // int totalHardConstraintSize = 0;
    // int frame, end;
    // if(isSingleFrameMode){
    //     frame = 0;
    //     end = 1;
    // } else {
    //     frame = 0;
    //     end = constraintInfoSeq.size();
    // }
    // while(frame < end){
    //     ConstraintInfo& cinfo = constraintInfoSeq[frame++];
    //     cinfo.globalHardConstraintIndex = totalHardConstraintSize;
    //     totalSoftConstraintSize += cinfo.softPositionalConstraintSize();
    //     totalHardConstraintSize += numAllBones + cinfo.hardPositionalConstraintSize();
    // }

    const int size = m3n; // + totalHardConstraintSize;
    //A.resize(size, size);
    //y.resize(size);
    //x.resize(size);

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


void LaplacianDeformationEnergy::extractBones()
{
    boneEdgeMap.clear();

    for(size_t i=0; i < characterInfos.size(); ++i){
        CharacterInfo& chara = characterInfos[i];
        chara.bones.clear();
        if(chara.org){
            const cnoid::MarkerMotionPtr& motion = mesh->motion(i);
            const int vertexIndexOffset = mesh->globalVertexIndexOffset(i);
            const int numBones = chara.org->numMarkerEdges();
            for(int j=0; j < numBones; ++j){
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

                        if(doExcludeBoneEdgesFromLaplacianCoordinate){
                            boneEdgeMap[pe1ActiveIndex].insert(pe2GlobalIndex);
                            boneEdgeMap[pe2ActiveIndex].insert(pe1GlobalIndex);
                        }
                    }
                }
            }
        }
    }
}


    void
    LaplacianDeformationEnergy::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

      result.setZero ();

      assert (result.size () == 1);

      // update interaction mesh.
      for (int frameId = 0; frameId < markerMotion_->numFrames (); ++frameId)
	{
	  cnoid::MarkerMotion::Frame frame = markerMotion_->frame (frameId);
	  for (int partId = 0; partId < markerMotion_->numParts (); ++partId)
	    frame[partId] =
	      x.segment<3> (frameId * markerMotion_->numParts () * 3 + partId * 3);
	}
      mesh->update ();

      // compute laplacian deformation energy.
      for (int currentFrame = 0; currentFrame < m; ++currentFrame)
	{
	  cnoid::MatrixXd& Mi = M[currentFrame];
	  cnoid::VectorXd& bi = b[currentFrame];
	  cnoid::VectorXd Vi(m3);
	  int index = 0;
	  for(int i=0; i < m; ++i){
	    const cnoid::MarkerIMesh::LocalIndex& lindex = mesh->activeToLocalIndex(i);
	    const cnoid::Vector3& v = Vi_frames[lindex.motionIndex][lindex.markerIndex];
	    Vi[index++] = v[0];
	    Vi[index++] = v[1];
	    Vi[index++] = v[2];
	    //Vi.segment<3>(i * 3) = Vi_frames[index.motionIndex][index.markerIndex];
	  }
	  bi = Mi * Vi;

	  double tmp = (Mi * x + bi).norm ();
	  result[0] += tmp * tmp;
	}

      result[0] *= .5;
    }

    void
    LaplacianDeformationEnergy::impl_gradient
    (gradient_t& gradient,
     const argument_t& x,
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
