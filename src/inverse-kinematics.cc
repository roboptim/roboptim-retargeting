#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/inverse-kinematics.hh"

#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>



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

      result.setZero ();

      for (std::size_t linkId = 0; linkId < model_->numLinks (); ++linkId)
	{
	  hrp::Link* link = model_->link (linkId);
	  assert (link && "invalid link");

	  link->p = x.segment(linkId * 3, 3);

	  hrp::Link* baseLink = link->parent;

	  if (!baseLink)
	    continue;

	  hrp::JointPathPtr jointPath =
	    model_->getJointPath(baseLink, link);
	  assert (jointPath && "invalid joint path");
	  jointPath->calcInverseKinematics
	    (baseLink->p, baseLink->R, link->p, link->R);
	  result[linkId - 1] = jointPath->joint (0)->q;
	}
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

