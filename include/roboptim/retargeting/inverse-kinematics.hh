#ifndef ROBOPTIM_RETARGETING_INVERSE_KINEMATICS_HH
# define ROBOPTIM_RETARGETING_INVERSE_KINEMATICS_HH
# include <boost/make_shared.hpp>

# include <hrpModel/ModelLoaderUtil.h>
# include <hrpModel/Link.h>

# include <boost/shared_ptr.hpp>
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
      static boost::shared_ptr<InverseKinematics>
      create (const std::string& filename)
      {
	hrp::BodyPtr model = boost::make_shared<hrp::Body> ();
	char* argv[] = {"foo"};
	int argc = 1;
	hrp::loadBodyFromModelLoader (model, filename.c_str (), argc, argv);
	model->calcForwardKinematics ();
	return boost::make_shared<InverseKinematics> (model);
      }

      explicit InverseKinematics
      (const hrp::BodyPtr& body)
	throw ();
      virtual ~InverseKinematics () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();

      const hrp::BodyPtr& model () const
      {
	return model_;
      }
      hrp::BodyPtr& model ()
      {
	return model_;
      }
    private:
      mutable hrp::BodyPtr model_;
      mutable Eigen::VectorXd Qinit_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_INVERSE_KINEMATICS_HH
