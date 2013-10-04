#ifndef ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_CHOREONOID_HH
# include <boost/format.hpp>

# include <cnoid/Body>
# include <cnoid/JointPath>

# include <roboptim/retargeting/function/forward-geometry.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Compute forward geometry for a particular robot model.
    ///
    /// Robot model from choreonoid should be passed to the
    /// constructor and use for computation.  The analytical jacobian
    /// must also be retrieved and returned in the
    /// impl_gradient/impl_jacobian methods.
    template <typename T>
    class ForwardGeometryChoreonoid : public ForwardGeometry<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_ (ForwardGeometry<T>);

      explicit ForwardGeometryChoreonoid
      (cnoid::BodyPtr robot, size_type bodyId) throw ()
	: ForwardGeometry<T> (robot->numJoints (), "choreonoid"),
	  robot_ (robot),
	  bodyId_ (bodyId),
	  jointPath_ (robot->rootLink ())
      {
	if (bodyId >= robot->numLinks ())
	  {
	    boost::format fmt
	      ("failed to construct ForwardGeometryChoreonoid function:"
	       " invalid body id %d (robot contains %d bodies)");
	    fmt % bodyId % robot->numLinks ();
	    throw std::runtime_error (fmt.str ());
	  }
      }

      explicit ForwardGeometryChoreonoid
      (cnoid::BodyPtr robot, const std::string& bodyName) throw ()
	: ForwardGeometry<T> (robot->numJoints (), "choreonoid"),
	  robot_ (robot),
	  bodyId_ (0),
	  jointPath_ (robot->rootLink ())
      {
	cnoid::Link* link = robot->link (bodyName.c_str ());
	if (!link)
	  {
	    boost::format fmt
	      ("failed to construct ForwardGeometryChoreonoid function:"
	       " no body whose name is '%s' can be found");
	    fmt % bodyName;
	    throw std::runtime_error (fmt.str ());
	  }

	bodyId_ = link->jointId ();
	if (bodyId_ >= robot->numLinks ())
	  {
	    boost::format fmt
	      ("failed to construct ForwardGeometryChoreonoid function:"
	       " invalid body id %d (robot contains %d bodies)");
	    fmt % bodyId_ % robot->numLinks ();
	    throw std::runtime_error (fmt.str ());
	  }
      }


      virtual ~ForwardGeometryChoreonoid () throw ()
      {}

    protected:

      // FIXME: once again we make a huge assumption here by
      // considering all joints are 1-DOFs joints.
      void
      impl_compute
      (result_t& result, const argument_t& x)
	const throw ()
      {
	for(int dofId = 0; dofId < robot_->numJoints (); ++dofId)
	  robot_->joint (dofId)->q () = x[dofId];
	robot_->calcForwardKinematics (true, true);
	result = robot_->link (bodyId_)->p ();
      }

      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type i)
	const throw ()
      {
	for(int dofId = 0; dofId < robot_->numJoints (); ++dofId)
	  robot_->joint (dofId)->q () = x[dofId];
	robot_->calcForwardKinematics (true, true);

	Eigen::MatrixXd matrix;
	jointPath_.calcJacobian (matrix);
	gradient = matrix;
      }

    private:
      cnoid::BodyPtr robot_;
      size_type bodyId_;
      mutable cnoid::JointPath jointPath_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_CHOREONOID_HH
