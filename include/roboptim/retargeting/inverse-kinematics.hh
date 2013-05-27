#ifndef ROBOPTIM_RETARGETING_INVERSE_KINEMATICS_HH
# define ROBOPTIM_RETARGETING_INVERSE_KINEMATICS_HH
# define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
# define EIGEN_RUNTIME_NO_MALLOC
# include <rbdl/rbdl.h>
# include <rbdl/Kinematics.h>
# include <rbdl/addons/urdfreader/rbdl_urdfreader.h>


# include <boost/shared_ptr.hpp>
# include <urdf_interface/model.h>
# include <rbdl/Model.h>
# include <roboptim/core/differentiable-function.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

namespace roboptim
{
  namespace retargeting
  {
    class DirectGeometry;
    typedef boost::shared_ptr<DirectGeometry> DirectGeometryShPtr_t;

    /// \brief Direct geometry computation using metpod.
    class InverseKinematics :
      public roboptim::GenericDifferentiableFunction<EigenMatrixDense>
    {
    public:
      explicit InverseKinematics
      (const std::string& filename,
       boost::shared_ptr<urdf::ModelInterface> model)
	throw ();
      virtual ~InverseKinematics () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();

      const RigidBodyDynamics::Model& rbdlModel () const
      {
	return rbdlModel_;
      }
      RigidBodyDynamics::Model& rbdlModel ()
      {
	return rbdlModel_;
      }
    private:
      boost::shared_ptr<urdf::ModelInterface> model_;
      mutable RigidBodyDynamics::Model rbdlModel_;
      std::vector<unsigned> bodyIds_;
      std::vector<RigidBodyDynamics::Math::Vector3d> bodyPoints_;
      mutable std::vector<RigidBodyDynamics::Math::Vector3d> targetPositions_;
      mutable RigidBodyDynamics::Math::VectorNd Qinit_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_INVERSE_KINEMATICS_HH
