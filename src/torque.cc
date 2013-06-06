#include <boost/fusion/algorithm/iteration/fold.hpp>
#include <boost/fusion/include/fold.hpp>
#include <boost/make_shared.hpp>

#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/torque.hh"

#include <metapod/models/simple_humanoid/simple_humanoid.hh>

#include "model/hrp4g2.hh"

#include <metapod/tools/print.hh>
#include <metapod/algos/rnea.hh>

#include <hrpModel/Link.h>

// Define which robot to use.
typedef metapod::hrp4g2 robot_t;

namespace roboptim
{
  namespace retargeting
  {
    namespace
    {
      struct getJointIdVisitor
      {
	typedef unsigned result_type;
	std::string jointName;

	template<typename T>
	result_type operator() (const unsigned& i, const T& node) const
	{
	  if (i !=-1)
	    return i;
	  if (node.joint_name == jointName)
	    return node.id;
	  return -1;
	}
      };

      static unsigned getJointId (const std::string& jointName)
      {
	robot_t robot;
	getJointIdVisitor visitor;
	visitor.jointName = jointName;
	unsigned res =
	  boost::fusion::fold (robot.nodes, -1, visitor);
	if (res == -1)
	  throw std::runtime_error ("joint lookup failed");
      }
    } // end of anonymous namespace.

    struct LinkInfo
    {
      std::string associatedMarker;
      Eigen::Vector3d offset;
    };
    std::map<std::string, LinkInfo> linkInfo_;

# define DECLARE_LINK_MAPPING(LINK_NAME, MARKER, OFFSET)		\
    do									\
      {									\
	LinkInfo link;							\
	link.associatedMarker = MARKER;					\
	link.offset = OFFSET;						\
	linkInfo_[LINK_NAME] = link;					\
      }									\
    while (0)								\

    Torque::Torque
    (AnimatedInteractionMeshShPtr_t animatedMesh,
     AnimatedInteractionMeshShPtr_t animatedMeshLocal,
     boost::shared_ptr<InverseKinematics> ik) throw ()
      : roboptim::GenericDifferentiableFunction<EigenMatrixSparse>
	(static_cast<size_type> (animatedMesh->optimizationVectorSize ()),
	 animatedMesh->numFrames () * robot_t::NBDOF, "torque"),
	animatedMesh_ (animatedMesh),
	animatedMeshLocal_ (animatedMeshLocal),
	torqueLimits_ (robot_t::NBDOF),
	ik_ (ik)
    {
      DECLARE_LINK_MAPPING("WAIST", "Chest", Eigen::Vector3d::Zero ());

      DECLARE_LINK_MAPPING("R_HIP_Y", "RightHip", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("R_HIP_R", "RightHip", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("R_HIP_P", "RightHip", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("R_KNEE_P", "RightKnee", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("R_ANKLE_P", "RightAnkle", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("R_ANKLE_R", "RightAnkle", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("R_TOE_P", "RightToe", Eigen::Vector3d::Zero ());

      DECLARE_LINK_MAPPING("L_HIP_Y", "LeftHip", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("L_HIP_R", "LeftHip", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("L_HIP_P", "LeftHip", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("L_KNEE_P", "LeftKnee", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("L_ANKLE_P", "LeftAnkle", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("L_ANKLE_R", "LeftAnkle", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("L_TOE_P", "LeftToe", Eigen::Vector3d::Zero ());

      DECLARE_LINK_MAPPING("CHEST_P", "Chest", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("CHEST_R", "Chest", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("CHEST_Y", "Chest", Eigen::Vector3d::Zero ());

      DECLARE_LINK_MAPPING("NECK_P", "Neck", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("NECK_R", "Neck", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("NECK_Y", "Neck", Eigen::Vector3d::Zero ());

      DECLARE_LINK_MAPPING("EYEBROW_P", "Chin", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("EYELID_P", "Chin", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("EYE_P", "Chin", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("EYE_Y", "Chin", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("MOUTH_P", "Chin", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("LOWERLIP_P", "Chin", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("UPPERLIP_P", "Chin", Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("CHEEK_P", "Chin", Eigen::Vector3d::Zero ());

      DECLARE_LINK_MAPPING("R_SHOULDER_P", "RightShoulder",
			   Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("R_SHOULDER_R", "RightShoulder",
			   Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("R_SHOULDER_Y", "RightShoulder",
			   Eigen::Vector3d::Zero ());

      DECLARE_LINK_MAPPING("R_ELBOW_P", "RightElbow",
			   Eigen::Vector3d::Zero ());

      DECLARE_LINK_MAPPING("R_WRIST_Y", "RightWristThumb",
			   Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("R_WRIST_R", "RightWristThumb",
			   Eigen::Vector3d::Zero ());

      DECLARE_LINK_MAPPING("R_HAND_J0", "RightWristThumb",
			   Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("R_HAND_J1", "RightWristThumb",
			   Eigen::Vector3d::Zero ());

      DECLARE_LINK_MAPPING("L_SHOULDER_P", "LeftShoulder",
			   Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("L_SHOULDER_R", "LeftShoulder",
			   Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("L_SHOULDER_Y", "LeftShoulder",
			   Eigen::Vector3d::Zero ());

      DECLARE_LINK_MAPPING("L_ELBOW_P", "LeftElbow",
			   Eigen::Vector3d::Zero ());

      DECLARE_LINK_MAPPING("L_WRIST_Y", "RightWristThumb",
			   Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("L_WRIST_R", "RightWristThumb",
			   Eigen::Vector3d::Zero ());

      DECLARE_LINK_MAPPING("L_HAND_J0", "RightWristThumb",
			   Eigen::Vector3d::Zero ());
      DECLARE_LINK_MAPPING("L_HAND_J1", "RightWristThumb",
			   Eigen::Vector3d::Zero ());
    }

    Torque::~Torque () throw ()
    {}

    void
    Torque::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

      animatedMeshLocal_->state () = x;
      animatedMeshLocal_->computeVertexWeights();

      robot_t robot;

      std::vector<robot_t::confVector> q (animatedMeshLocal_->numFrames ());
      std::vector<robot_t::confVector> dq (animatedMeshLocal_->numFrames ());
      std::vector<robot_t::confVector> ddq (animatedMeshLocal_->numFrames ());

      // Compute q from segment positions.
      for (unsigned frameId = 0; frameId < animatedMeshLocal_->numFrames ();
	   ++frameId)
	{
	  argument_t bodyPositions (ik_->model ()->numLinks () * 3);

	  for (std::size_t linkId = 0; linkId < ik_->model ()->numLinks ();
	       ++linkId)
	    {
	      hrp::Link* link = ik_->model ()->link (linkId);

	      std::string markerName =
		linkInfo_[link->name].associatedMarker;
	      assert (!markerName.empty ());

	      AnimatedInteractionMesh::vertex_iterator_t vertex =
		animatedMeshLocal_->getVertexFromLabel (markerName);

	      AnimatedInteractionMesh::vertex_iterator_t vertexIt;
	      AnimatedInteractionMesh::vertex_iterator_t vertexEnd;
	      boost::tie (vertexIt, vertexEnd) =
		boost::vertices (animatedMeshLocal_->graph ());
	      assert (vertex != vertexEnd);

	      bodyPositions.segment (linkId * 3, 3) =
		animatedMeshLocal_->graph ()[*vertex].positions[frameId];

	      bodyPositions.segment (linkId * 3, 3) +=
		linkInfo_[link->name].offset;
	    }

	  q[frameId] = (*ik_) (bodyPositions);
	}

      // Fill dq with frameId = 0 to n - 1.
      for (unsigned frameId = 0; frameId < animatedMeshLocal_->numFrames () - 1;
	   ++frameId)
	{
	  for (unsigned dofId = 0; dofId < robot_t::NBDOF; ++dofId)
	    dq[frameId][dofId] =
	      (q[frameId][dofId] - q[frameId + 1][dofId])
	      / animatedMeshLocal_->framerate ();
	  dq[frameId][animatedMeshLocal_->numFrames () - 1] = 0.;
	}

      // Fill ddq with frameId = 1 to n - 1
      for (unsigned frameId = 1; frameId < animatedMeshLocal_->numFrames () - 1;
	   ++frameId)
	{
	  for (unsigned dofId = 0; dofId < robot_t::NBDOF; ++dofId)
	    ddq[frameId][dofId] =
	      (q[frameId + 1][dofId]
	     - 2 * q[frameId][dofId]
	       - q[frameId - 1][dofId])
	      / (animatedMeshLocal_->framerate ()
		 * animatedMeshLocal_->framerate ());
	  dq[frameId][0] = 0.;
	  dq[frameId][animatedMeshLocal_->numFrames () - 1] = 0.;
	}

      robot_t::confVector torques;
      for (unsigned frameId = 1; frameId < animatedMeshLocal_->numFrames () - 1;
	   ++frameId)
	{
	  metapod::rnea<robot_t, true>::run
	    (robot, q[frameId], dq[frameId], ddq[frameId]);
	  metapod::getTorques (robot, torques);

	  result.segment
	    (frameId * robot_t::NBDOF, robot_t::NBDOF) = torques;
	}
    }

    void
    Torque::impl_gradient
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
