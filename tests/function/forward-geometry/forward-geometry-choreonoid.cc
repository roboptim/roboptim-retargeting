#include <boost/format.hpp>
#include <boost/make_shared.hpp>

#include <roboptim/core/terminal-color.hh>
#include <roboptim/retargeting/function/minimum-jerk-trajectory.hh>
#include <roboptim/retargeting/function/forward-geometry/choreonoid.hh>
#include <roboptim/trajectory/vector-interpolation.hh>

#include <cnoid/BodyLoader>

#define BOOST_TEST_MODULE forward_geometry_chorenoid

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

using boost::test_tools::output_test_stream;

using namespace roboptim;
using namespace roboptim::retargeting;

std::string modelFilePath (HRP4C_YAML_FILE);

BOOST_AUTO_TEST_CASE (root_link)
{
  // Loading robot.
  cnoid::BodyLoader loader;
  cnoid::BodyPtr robot = loader.load (modelFilePath);
  if (!robot)
    throw std::runtime_error ("failed to load model");

  typedef ForwardGeometryChoreonoid<EigenMatrixDense>::jacobian_t jacobian_t;
  typedef ForwardGeometryChoreonoid<EigenMatrixDense>::vector_t vector_t;
  ForwardGeometryChoreonoid<EigenMatrixDense> forwardGeometry (robot, 0);

  vector_t x (6 + robot->numJoints ());
  vector_t res (6);

  // Zero
  x.setZero ();
  res = forwardGeometry (x);
  std::cout
    << "X:" << incindent << iendl
    << x << decindent << iendl
    << "ForwardGeometry(X): " << incindent << iendl
    << res << decindent << iendl;

  for (vector_t::Index i = 0; i < 6; ++i)
    BOOST_CHECK_EQUAL (res[i], 0.);

  // 1, 2, 3, Zero*N
  x.setZero ();
  x[0] = 1.;
  x[1] = 4.;
  x[2] = 8.;
  x[3] = M_PI / 2.;
  x[4] = M_PI;
  x[5] = 1.5 * M_PI;
  res = forwardGeometry (x);
  std::cout
    << "X:" << incindent << iendl
    << x << decindent << iendl
    << "ForwardGeometry(X): " << incindent << iendl
    << res << decindent << iendl;

  for (vector_t::Index i = 0; i < 3; ++i)
    BOOST_CHECK_CLOSE (res[i], x[i], 1e-5);

  // Jacobian is identity for free-floating.
  x.setRandom (x.size ());
  std::cout
    << "X:" << incindent << iendl
    << x << decindent << iendl;
  jacobian_t jacobian = forwardGeometry.jacobian (x);
  std::cout
    << "ForwardGeometry.jacobian(X): " << incindent << iendl
    << jacobian << decindent << iendl;

  for (vector_t::Index i = 0; i < 6; ++i)
    BOOST_CHECK_CLOSE (jacobian (i, i), 1., 1e-5);
}

#define CHECK_GRADIENT(F, I, X)						\
  BOOST_CHECK_NO_THROW							\
  (									\
   try									\
     {									\
       checkGradientAndThrow ((F), (I), (X));				\
     }									\
   catch(const BadGradient<EigenMatrixDense>& e)			\
     {									\
       std::cerr << #F << " (" << I << "):\n" << e << std::endl;	\
       throw;								\
     } )

#define CHECK_JACOBIAN(F, X)				\
  BOOST_CHECK_NO_THROW					\
  (							\
   try							\
     {							\
       checkJacobianAndThrow ((F), (X));		\
     }							\
   catch(const BadJacobian<EigenMatrixDense>& e)	\
     {							\
       std::cerr << #F << ":\n" << e << std::endl;	\
       throw;						\
     } )


template <typename T>
void displayMatrix (std::ostream& o, const T& m)
{
  int w = 3;
  o << std::setprecision (w - 1);
  int nEltsPerLine = 80 / (2 * w);

  typename T::Index offset = 0;

  while (offset < m.cols ())
    {
      for (typename T::Index i = 0; i < 6; ++i)
	{
	  for (typename T::Index j = offset; j < offset + nEltsPerLine; ++j)
	    {
	      if (j >= m.cols ())
		break;

	      if (j < 3)
		o << roboptim::fg::green;
	      else if (j < 6)
		o << roboptim::fg::red;
	      else if (i >= 3)
		o << roboptim::fg::orange;

	      o << boost::format ("% 4.2f") % m (i, j) << ",";
	      o << roboptim::fg::reset;
	    }
	  o << iendl;
	}

      o << iendl;
      offset += nEltsPerLine;
    }
  o << iendl;
}

BOOST_AUTO_TEST_CASE (lleg_link)
{
  // Loading robot.
  cnoid::BodyLoader loader;
  cnoid::BodyPtr robot = loader.load (modelFilePath);
  if (!robot)
    throw std::runtime_error ("failed to load model");

  typedef ForwardGeometryChoreonoid<EigenMatrixDense>::jacobian_t jacobian_t;
  typedef ForwardGeometryChoreonoid<EigenMatrixDense>::vector_t vector_t;
  ForwardGeometryChoreonoid<EigenMatrixDense> forwardGeometry (robot, "L_ANKLE_R");

  GenericFiniteDifferenceGradient<EigenMatrixDense> fdfunction (forwardGeometry);

  vector_t x (6 + robot->numJoints ());
  vector_t res (6);
  jacobian_t jacobian (6, x.size ());
  jacobian_t jacobianFd (6, x.size ());

  // Check jacobian
  for (std::size_t trial = 0; trial < 1; ++trial)
    {
      //x.setRandom ();
      x.setZero ();
      //x.segment(0, 3).setRandom ();
      //x[1] = 1.;
      //x[0] = 1.;
      x[4] = 1.;

      forwardGeometry (res, x);
      forwardGeometry.jacobian (jacobian, x);
      fdfunction.jacobian (jacobianFd, x);

      std::cout
	<< "X:" << incindent << iendl
	<< x << decindent << iendl
	<< "ForwardGeometry(X): " << incindent << iendl
	<< res << decindent << iendl
	<< "ForwardGeometry(X) Jacobian: " << iendl;

      displayMatrix (std::cout, jacobian);

      std::cout << "Finite Differences Jacobian: " << iendl;
      displayMatrix (std::cout, jacobianFd);

      for (vector_t::Index functionId = 0; functionId < 3; ++functionId)
	CHECK_GRADIENT (forwardGeometry, functionId, x);
    }
}
