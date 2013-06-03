#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_RUNTIME_NO_MALLOC
#include <rbdl/rbdl.h>
#include <rbdl/Kinematics.h>
#include <rbdl/addons/urdfreader/rbdl_urdfreader.h>

#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Link.h>


#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/inverse-kinematics.hh"


namespace roboptim
{
  namespace retargeting
  {
    InverseKinematics::InverseKinematics
    (const hrp::BodyPtr& model)
      throw ()
      : roboptim::GenericDifferentiableFunction<EigenMatrixDense>
	(model->numLinks () * 3,
	 model->numJoints (), "inverse kinematics"),
	model_ (model),
	Qinit_ ()
    {
      // FIXME: initialize Qinit to half-sitting?
      Qinit_.resize (model_->numJoints());
    }

    InverseKinematics::~InverseKinematics () throw ()
    {}

    void
    InverseKinematics::impl_compute
    (result_t& result, const argument_t& x)
      const throw ()
    {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

      // for (unsigned i = 0; i < targetPositions_.size (); ++i)
      // 	targetPositions_[i] = x.segment (i * 3, 3);

      // if (!RigidBodyDynamics::InverseKinematics
      // 	  (rbdlModel_, Qinit_, bodyIds_, bodyPoints_, targetPositions_,
      // 	   result, 1e-12, 0.01, 200))
      // 	throw std::runtime_error ("IK failed");
    }

    void
    InverseKinematics::impl_gradient
    (gradient_t& gradient,
     const argument_t& x,
     size_type i)
      const throw ()
    {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

      roboptim::GenericFiniteDifferenceGradient<
	EigenMatrixDense,
	finiteDifferenceGradientPolicies::Simple<EigenMatrixDense> >
	fdg (*this);
      fdg.gradient (gradient, x, i);
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.

