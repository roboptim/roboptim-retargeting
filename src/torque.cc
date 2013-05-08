#include <boost/fusion/algorithm/iteration/fold.hpp>
#include <boost/fusion/include/fold.hpp>

#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/torque.hh"

#include <metapod/models/simple_humanoid/simple_humanoid.hh>

#include "model/hrp4g2.hh"

#include <metapod/tools/print.hh>
#include <metapod/algos/rnea.hh>

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

      //FIXME: for now edit the code here to change the joints.
    static const char* consideredDofsNames[] =
      {
	"L_HIP_Y",
	"L_HIP_P",
	"L_HIP_R",

	"R_HIP_Y",
	"R_HIP_P",
	"R_HIP_R",
	0
      };

    Torque::Torque
    (boost::shared_ptr<urdf::ModelInterface> model,
     AnimatedInteractionMeshShPtr_t animatedMesh,
     AnimatedInteractionMeshShPtr_t animatedMeshLocal) throw ()
      : roboptim::GenericDifferentiableFunction<EigenMatrixSparse>
	(static_cast<size_type> (animatedMesh->optimizationVectorSize ()),
	 animatedMeshLocal_->numFrames () * robot_t::NBDOF, "torque"),
	model_ (model),
	animatedMesh_ (animatedMesh),
	animatedMeshLocal_ (animatedMeshLocal),
	consideredDofs_ (sizeof (consideredDofsNames)),
	torqueLimits_ (sizeof (consideredDofsNames))
    {

      for (unsigned i = 0; i < sizeof (consideredDofsNames); ++i)
	{
	  consideredDofs_[i] = getJointId (consideredDofsNames[i]);

	  boost::shared_ptr<const urdf::Joint> joint =
	    model_->getJoint (consideredDofsNames[i]);
	  if (!joint)
	    throw std::runtime_error ("joint lookup failed in URDF model");
	  boost::shared_ptr<const urdf::JointLimits> limits =
	    joint->limits;
	  if (!limits)
	    throw std::runtime_error ("missing limit in URDF model");
	  torqueLimits_[i] = std::make_pair (-limits->effort, limits->effort);
	}

      // Associate DOFs with segments.
      std::map<std::pair<std::string, std::string>, std::string> map;

      map[std::make_pair ("a", "b")] = "c";
    }

    Torque::~Torque () throw ()
    {}

    void
    Torque::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
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
	for (unsigned dofId = 0; dofId < robot_t::NBDOF; ++dofId)
	  {
	    // 1. which segment are related to this dof?
	    AnimatedInteractionMesh::vertex_descriptor_t segment1  = 0;
	    AnimatedInteractionMesh::vertex_descriptor_t segment2  = 0;

	    // 2. get segment positions
	    const Vertex::position_t& segment1Position =
	      animatedMeshLocal_->graph ()[segment1].positions[frameId];
	    const Vertex::position_t& segment2Position =
	      animatedMeshLocal_->graph ()[segment2].positions[frameId];

	    // 3. identify articular value
	    q[frameId][dofId] =
	      std::asin
	      (segment1Position.cross (segment2Position)[0]
	       / segment1Position.norm ()
	       / segment2Position.norm ());
	  }
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
      roboptim::GenericFiniteDifferenceGradient<
	EigenMatrixSparse,
	finiteDifferenceGradientPolicies::Simple<EigenMatrixSparse> >
	fdg (*this);
      fdg.gradient (gradient, x, i);
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
