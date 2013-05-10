#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_RUNTIME_NO_MALLOC
#include <rbdl/rbdl.h>
#include <rbdl/Kinematics.h>
#include <rbdl/addons/urdfreader/rbdl_urdfreader.h>

#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/inverse-kinematics.hh"


namespace roboptim
{
  namespace retargeting
  {
    InverseKinematics::InverseKinematics
    (const std::string& filename,
     boost::shared_ptr<urdf::ModelInterface> model)
      throw ()
      : roboptim::GenericDifferentiableFunction<EigenMatrixDense>
	(model->links_.size () * 3,
	 model->joints_.size (), "inverse kinematics"),
	model_ (model),
	rbdlModel_ (),
	bodyIds_ (model->links_.size ()),
	bodyPoints_ (model->links_.size ()),
	targetPositions_ (model->links_.size ()),
	Qinit_ (model->joints_.size ())
    {
      // load the URDF model using RBDL
      if (!RigidBodyDynamics::Addons::read_urdf_model
	  (filename.c_str (), &rbdlModel_, true))
	throw std::runtime_error ("RBDL failed to load the URDF model");

      for (unsigned i = 0; i < model->links_.size (); ++i)
	{
	  bodyIds_[i] = i;
	  bodyPoints_[i].setZero ();
	  targetPositions_[i].setZero ();
	}

      // FIXME: initialize Qinit to half-sitting?
    }

    InverseKinematics::~InverseKinematics () throw ()
    {}

    void
    InverseKinematics::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
      for (unsigned i = 0; i < targetPositions_.size (); ++i)
	targetPositions_[i] = x.segment (i * 3, 3);

      if (!RigidBodyDynamics::InverseKinematics
	  (rbdlModel_, Qinit_, bodyIds_, bodyPoints_, targetPositions_,
	   result, 1e-12, 0.9, 50))
	throw std::runtime_error ("IK failed");
    }

    void
    InverseKinematics::impl_gradient
    (gradient_t& gradient,
     const argument_t& x,
     size_type i)
      const throw ()
    {
      roboptim::GenericFiniteDifferenceGradient<
	EigenMatrixDense,
	finiteDifferenceGradientPolicies::Simple<EigenMatrixDense> >
	fdg (*this);
      fdg.gradient (gradient, x, i);
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.

