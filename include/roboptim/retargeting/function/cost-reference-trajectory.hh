#ifndef ROBOPTIM_RETARGETING_FUNCTION_COST_REFERENCE_TRAJECTORY_HH
# define ROBOPTIM_RETARGETING_FUNCTION_COST_REFERENCE_TRAJECTORY_HH
# include <string>

# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    template <typename T>
    class CostReferenceTrajectory : public GenericDifferentiableFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericDifferentiableFunction<T>);

      /// \brief Import discrete interval type.
      typedef typename parent_t::discreteInterval_t discreteInterval_t;
      /// \brief Import discrete interval type.
      typedef typename parent_t::interval_t interval_t;

      typedef Trajectory<3> trajectory_t;

      explicit CostReferenceTrajectory
      (boost::shared_ptr<Trajectory<3> > referenceTrajectory,
       size_type dofId,
       value_type dt)
	throw ()
	: GenericDifferentiableFunction<T>
	  (referenceTrajectory->parameters ().size (), 1, "CostReferenceTrajectory"),
	  vectorInterpolation_
	  (boost::make_shared<VectorInterpolation>
	   (vector_t::Zero (referenceTrajectory->parameters ().size ()),
	    referenceTrajectory->outputSize (), dt)),
	  referenceTrajectory_ (referenceTrajectory),
	  dofId_ (dofId),
	  reference_ (referenceTrajectory->outputSize ()),
	  value_ (referenceTrajectory->outputSize ()),
	  dt_ (dt)
      {}

      virtual ~CostReferenceTrajectory () throw ()
      {}

    protected:
      virtual void impl_compute (result_t& result,
				 const argument_t& p)
	const throw ()
      {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	vectorInterpolation_->setParameters (p);
	const value_type min = referenceTrajectory_->timeRange ().first;
	const value_type max = referenceTrajectory_->timeRange ().second;
	for (value_type t = min; t < max; t += dt_)
	  {
	    (*vectorInterpolation_) (value_, t);
	    (*referenceTrajectory_) (reference_, t);
	    result[0] +=
	      .5 * (value_[dofId_] - reference_[dofId_])
	      * (value_[dofId_] - reference_[dofId_]);
	  }
      }

      virtual void impl_gradient (gradient_t& gradient,
				  const argument_t& p,
				  size_type)
	const throw ()
      {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	vectorInterpolation_->setParameters (p);

	const value_type min = referenceTrajectory_->timeRange ().first;
	const value_type max = referenceTrajectory_->timeRange ().second;

	for (value_type t = min; t < max; t += dt_)
	  {
	    (*vectorInterpolation_) (value_, t);
	    (*referenceTrajectory_) (reference_, t);

	    gradient +=
	      (value_[dofId_] - reference_[dofId_])
	      * vectorInterpolation_->variationStateWrtParam (t, 1).row (dofId_);
	  }
      }

    private:
      boost::shared_ptr<VectorInterpolation > vectorInterpolation_;
      boost::shared_ptr<Trajectory<3> > referenceTrajectory_;
      size_type dofId_;
      mutable result_t reference_;
      mutable result_t value_;
      value_type dt_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_COST_REFERENCE_TRAJECTORY_HH
