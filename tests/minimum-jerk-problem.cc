// Copyright (C) 2013 by Thomas Moulard, AIST, CNRS, INRIA.
//
// This file is part of the roboptim.
//
// roboptim is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// roboptim is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with roboptim.  If not, see <http://www.gnu.org/licenses/>.


#define BOOST_TEST_MODULE minimum_jerk_problem

#include <vector>
#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include "tests-config.h"

#include <roboptim/core/linear-function.hh>
#include <roboptim/core/optimization-logger.hh>
#include <roboptim/core/problem.hh>
#include <roboptim/core/solver.hh>
#include <roboptim/core/solver-factory.hh>
#include <roboptim/core/filter/chain.hh>
#include <roboptim/core/filter/selection.hh>
#include <roboptim/core/filter/split.hh>
#include <roboptim/core/visualization/gnuplot.hh>
#include <roboptim/core/visualization/gnuplot-commands.hh>
#include <roboptim/core/visualization/gnuplot-function.hh>
#include <roboptim/trajectory/state-function.hh>
#include <roboptim/trajectory/vector-interpolation.hh>

#include <roboptim/retargeting/function/minimum-jerk-trajectory.hh>
#include <roboptim/retargeting/function/forward-geometry/choreonoid.hh>
#include <roboptim/retargeting/function/torque/choreonoid.hh>
#include <roboptim/retargeting/function/torque/metapod.hh>
#include <roboptim/retargeting/function/zmp/choreonoid.hh>
#include <roboptim/retargeting/function/zmp/metapod.hh>

#include <cnoid/BodyLoader>
#include "model/hrp4g2.hh"

using namespace roboptim;
using namespace roboptim::visualization;
using namespace roboptim::retargeting;

using boost::test_tools::output_test_stream;

boost::array<double, 6 + 44> standardPose = {{
    0, 0, 0, 0, 0, 0,

    0, 0, -25, 50, -25, 0, 0,
    0, 0, -25, 50, -25, 0, 0,
    0, 0, 0,
    0, 0, 0,
    -1.0, 1.0, 0, 0, 0, -1.0, 1.0, -1.0,
    5, -10, 0, -15, 0,  10,  1.0, 0,
    5,  10, 0, -15, 0, -10, -1.0, 0
  }};

namespace roboptim
{
  template <typename T, typename U>
  class MinimumJerkOptimizationLogger : public OptimizationLogger<T>
  {
  public:
    typedef T solver_t;
    typedef typename solver_t::problem_t problem_t;
    typedef typename solver_t::problem_t::value_type value_type;
    typedef typename solver_t::problem_t::vector_t vector_t;
    typedef typename solver_t::problem_t::function_t::discreteInterval_t
    discreteInterval_t;

    explicit MinimumJerkOptimizationLogger
    (solver_t& solver, const boost::filesystem::path& path,
     discreteInterval_t discreteInterval,
     boost::shared_ptr<Trajectory<3> > trajectory,
     boost::shared_ptr<ZMP<U> > zmp)
      : OptimizationLogger<T> (solver, path),
	discreteInterval_ (discreteInterval),
	trajectory_ (trajectory),
	zmp_ (zmp)
    {}

    ~MinimumJerkOptimizationLogger () throw ()
    {}
  protected:
    virtual
    void perIterationCallbackUnsafe (const typename solver_t::vector_t& x,
				     const typename solver_t::problem_t& pb)
    {
      OptimizationLogger<T>::perIterationCallbackUnsafe (x, pb);

      boost::filesystem::path path = this->path ();
      const boost::filesystem::path iterationPath =
	path / (boost::format ("iteration-%d") % this->callbackCallId ()).str ();

      // Log ZMP
      if (zmp_)
	{
	  boost::filesystem::ofstream streamZmp (iterationPath / "zmp.csv");
	  streamZmp << "ZMP X, ZMP Y\n";

	  const value_type tmin = boost::get<0> (discreteInterval_);
	  const value_type tmax = boost::get<1> (discreteInterval_);
	  const value_type dt = boost::get<2> (discreteInterval_);

	  for (value_type t = tmin; t + dt < tmax; t += dt)
	    {
	      trajectory_->setParameters (x);
	      vector_t state = trajectory_->state (t, 2);
	      vector_t zmp = (*zmp_) (state);
	      streamZmp << zmp[0] << ", " << zmp[1] << "\n";
	    }

	  for (value_type  i = 0; i < x.size (); ++i)
	    {	    }
	  streamZmp << "\n";
	}
     }

   private:
     discreteInterval_t discreteInterval_;
     boost::shared_ptr<Trajectory<3> > trajectory_;
    boost::shared_ptr<ZMP<U> > zmp_;
  };
} // end of namespace roboptim.

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

typedef roboptim::Solver<
  GenericDifferentiableFunction<EigenMatrixDense>,
  boost::mpl::vector<
    GenericLinearFunction<EigenMatrixDense>,
    GenericDifferentiableFunction<EigenMatrixDense> >
  >
solver_t;
typedef solver_t::problem_t problem_t;
typedef boost::shared_ptr<problem_t> ProblemShPtr_t;

BOOST_AUTO_TEST_CASE (simple)
{
  // Forward typedefs.
  typedef MinimumJerkTrajectory<EigenMatrixDense>::vector_t vector_t;
  typedef MinimumJerkTrajectory<EigenMatrixDense>::interval_t interval_t;
  typedef MinimumJerkTrajectory<EigenMatrixDense>::value_type value_type;
  typedef MinimumJerkTrajectory<EigenMatrixDense>::size_type size_type;

  // Problem configuration
  // - number of frames
  std::size_t nFrames = 10.;
  // - delta t used for time synchronization
  value_type dt = 0.005;
  // - minimum trajectory time
  value_type tmin = 0.;
  // - maximum trajectory time
  value_type tmax = dt * static_cast<value_type> (nFrames);
  // - the id of the dof we move.
  std::size_t dofId = 0;
  // - dof initial position
  value_type init = standardPose[dofId] * M_PI / 360.;
  // - dof goal position
  value_type goal = (standardPose[dofId] + 1.) * M_PI / 360.;
  // - total number of dofs
  std::size_t nDofs = standardPose.size ();

  // Configure log4cxx
  configureLog4cxx ();

  // Compute the initial trajectory (whole body)
  vector_t initialTrajectory (nFrames * standardPose.size ());
  initialTrajectory.setZero ();

  vector_t x (4);
  x[0] = init;
  x[1] = goal;
  x[2] = x[3] = 0.;

  boost::shared_ptr<MinimumJerkTrajectory<EigenMatrixDense> >
    minimumJerkTrajectory =
    boost::make_shared<MinimumJerkTrajectory<EigenMatrixDense> >
    ();
  minimumJerkTrajectory->setParameters (x);

  // Build the starting point
  for (std::size_t frameId = 0; frameId < nFrames; ++frameId)
    {
      for (std::size_t dof = 0; dof < nDofs; ++dof)
	initialTrajectory[frameId * nDofs + dof] =
	  standardPose[dof] * M_PI / 360.;

      initialTrajectory[frameId * nDofs + dofId] =
	(*minimumJerkTrajectory)
	(static_cast<value_type> (frameId) * dt)[0];
    }

  // Build a trajectory over the starting point for display
  MinimumJerkTrajectory<EigenMatrixDense>::discreteInterval_t
    intervalS (tmin, tmax, 0.01);

  boost::shared_ptr<VectorInterpolation >
    initialTrajectoryFct =
    vectorInterpolation
    (initialTrajectory, static_cast<size_type> (nDofs), dt);

  // Build the cost function as the difference between the
  // reference trajectory and the current trajectory.
  boost::shared_ptr<CostReferenceTrajectory<EigenMatrixDense> >
    cost = boost::make_shared<CostReferenceTrajectory<EigenMatrixDense> >
    (initialTrajectoryFct, dofId, dt);

  // Clone the vector interpolation object so that it can be used by
  // constraints
  boost::shared_ptr<VectorInterpolation >
    vectorInterpolationConstraints =
    vectorInterpolation
    (initialTrajectory, static_cast<size_type> (nDofs), dt);


  // Create problem.
  ProblemShPtr_t problem = boost::make_shared<problem_t> (*cost);
  problem->startingPoint () = initialTrajectory;

  //FIXME: we should embed the copy.
  std::string modelFilePath
    ("/home/moulard/HRP4C-release/HRP4Cg2.yaml");

  // Loading robot.
  cnoid::BodyLoader loader;
  cnoid::BodyPtr robot = loader.load (modelFilePath);
  if (!robot)
    throw std::runtime_error ("failed to load model");

  // Add constraints.
  bool enableFeetPositionsConstraint = true;
  bool enableTorqueConstraint = false;
  bool enableZmpConstraint = true;

  // Should we use Metapod or Choreonoid?
  bool useMetapod = false;

  if (enableFeetPositionsConstraint)
    {
      unsigned nConstraints = 3;

      typedef ForwardGeometryChoreonoid<EigenMatrixDense> forwardGeometry_t;

      // Left foot.
      boost::shared_ptr<GenericDifferentiableFunction<EigenMatrixDense> >
	leftFootOneFrame =
	boost::make_shared<forwardGeometry_t>
	(robot, std::string ("L_TOE_P"));

      forwardGeometry_t::vector_t leftFootPosition =
	(*leftFootOneFrame) ((*initialTrajectoryFct) (0.));
      std::vector<interval_t> leftFootBounds (leftFootPosition.size ());
      for (std::size_t i = 0; i < leftFootPosition.size (); ++i)
	leftFootBounds[i] = forwardGeometry_t::makeInterval
	  (leftFootPosition[i] - 1e-4, leftFootPosition[i] + 1e-4);

      std::vector<value_type> leftFootScales (6);
      for (std::size_t i = 0; i < 6; ++i)
	leftFootScales[i] = 1.;

      roboptim::StateFunction<VectorInterpolation>::addToProblem
	(*vectorInterpolationConstraints, leftFootOneFrame, 0,
	 *problem, leftFootBounds, leftFootScales, nConstraints);

      // Right foot.
      boost::shared_ptr<GenericDifferentiableFunction<EigenMatrixDense> >
	rightFootOneFrame =
	boost::make_shared<forwardGeometry_t>
	(robot, std::string ("R_TOE_P"));

      forwardGeometry_t::vector_t rightFootPosition =
	(*rightFootOneFrame) ((*initialTrajectoryFct) (0.));
      std::vector<interval_t> rightFootBounds (rightFootPosition.size ());
      for (std::size_t i = 0; i < rightFootPosition.size (); ++i)
	rightFootBounds[i] = forwardGeometry_t::makeInterval
	  (rightFootPosition[i] - 1e-4, rightFootPosition[i] + 1e-4);
      std::vector<value_type> rightFootScales (6);
      for (std::size_t i = 0; i < 6; ++i)
	rightFootScales[i] = 1.;

      roboptim::StateFunction<VectorInterpolation>::addToProblem
	(*vectorInterpolationConstraints, rightFootOneFrame, 0,
	 *problem, rightFootBounds, rightFootScales, nConstraints);
    }

  boost::shared_ptr<GenericDifferentiableFunction<EigenMatrixDense> >
    torqueOneFrame;
  if (enableTorqueConstraint)
    {
      unsigned nConstraints = 1;
      std::vector<interval_t> torqueBounds (nDofs);
      for (std::size_t i = 0; i < nDofs; ++i)
	torqueBounds[i] = interval_t (-1, 1);

      std::vector<value_type> torqueScales (nDofs);
      for (std::size_t i = 0; i < nDofs; ++i)
	torqueScales[i] = 1.;

      if (useMetapod)
	torqueOneFrame =
	  boost::make_shared<TorqueMetapod<EigenMatrixDense, metapod::hrp4g2> > ();
      else
	torqueOneFrame =
	  boost::make_shared<TorqueChoreonoid<EigenMatrixDense> > (robot);

      roboptim::StateFunction<VectorInterpolation>::addToProblem
	(*vectorInterpolationConstraints, torqueOneFrame, 2,
	 *problem, torqueBounds, torqueScales, nConstraints);
    }

  boost::shared_ptr<ZMP<EigenMatrixDense> > zmpOneFrame;
  if (enableZmpConstraint)
    {
      unsigned nConstraints = 10;
      value_type soleX = 0.03;
      value_type soleY = 0.;
      value_type soleLength = 0.2; //FIXME:
      value_type soleWidth = 0.1; //FIXME:

      std::vector<interval_t> zmpBounds (2);
      zmpBounds[0] = interval_t (soleX - .5 * soleLength,
				 soleX + .5 * soleLength);
      zmpBounds[1] = interval_t (soleY - .5 * soleWidth,
				 soleY + .5 * soleWidth);

      std::vector<value_type> zmpScales (2);
      zmpScales[0] = 1.;
      zmpScales[1] = 1.;

      if (useMetapod)
	zmpOneFrame =
	  boost::make_shared<ZMPMetapod<EigenMatrixDense, metapod::hrp4g2> > ();
      else
	zmpOneFrame =
	  boost::make_shared<ZMPChoreonoid<EigenMatrixDense> > (robot);

      roboptim::StateFunction<VectorInterpolation>::addToProblem
	(*vectorInterpolationConstraints, zmpOneFrame, 2,
	 *problem, zmpBounds, zmpScales, nConstraints);
    }

  //FIXME: freeze start and end positions
  //FIXME: add position and velocity limits.

  // Solve problem.
  roboptim::SolverFactory<solver_t> factory ("cfsqp", *problem);
  solver_t& solver = factory ();


  MinimumJerkOptimizationLogger<solver_t, EigenMatrixDense> logger
    (solver, "/tmp/minimum-jerk-problem",
     intervalS, initialTrajectoryFct, zmpOneFrame);

  // Set solver parameters.
  solver.parameters ()["max-iterations"].value = 100;
  solver.parameters ()["ipopt.output_file"].value =
    "/tmp/ipopt.log";
  solver.parameters ()["ipopt.expect_infeasible_problem"].value = "yes";
  solver.parameters ()["ipopt.nlp_scaling_method"].value = "none";
  solver.parameters ()["nag.verify-level"].value = 0;

  std::cerr << "Solver:\n" << solver << std::endl;
  std::cerr << "start solving..." << std::endl;
  solver.solve ();
  std::cerr << "problem solved." << std::endl;

  // Rebuild final trajectory.
  std::cerr << "result type: " << solver.minimum ().type ().name () << std::endl;

  if (solver.minimum ().which () == solver_t::SOLVER_ERROR)
    {
      std::cerr << "error" << std::endl;
      roboptim::SolverError error =
  	boost::get<roboptim::SolverError> (solver.minimum ());
      std::cerr << "Result:\n" << error << std::endl;
      return;
    }

  boost::shared_ptr<VectorInterpolation >
    finalTrajectoryFct;

  if (solver.minimum ().which () == solver_t::SOLVER_VALUE_WARNINGS)
    {
      std::cerr << "warnings" << std::endl;
      roboptim::ResultWithWarnings result =
  	boost::get<roboptim::ResultWithWarnings> (solver.minimum ());
      std::cerr << "Result:\n" << result << std::endl;
      finalTrajectoryFct =
	vectorInterpolation
	(result.x, static_cast<size_type> (nDofs), dt);
    }

  if (solver.minimum ().which () == solver_t::SOLVER_VALUE)
    {
      std::cerr << "ok" << std::endl;
      roboptim::Result result =
	boost::get<roboptim::Result> (solver.minimum ());
      std::cerr << "Result:\n" << result << std::endl;
      finalTrajectoryFct =
	vectorInterpolation
	(result.x, static_cast<size_type> (nDofs), dt);
    }

  assert (!!finalTrajectoryFct);

  // Display initial and final trajectory.
  initialTrajectoryFct->setParameters (initialTrajectory);
  using namespace roboptim::visualization::gnuplot;
  Gnuplot gnuplot = Gnuplot::make_interactive_gnuplot ();
  std::cout
    << (gnuplot
	<< set ("multiplot layout 3, 1")
	<< plot (*minimumJerkTrajectory, intervalS)
	<< plot (*initialTrajectoryFct, intervalS)
	<< plot (*finalTrajectoryFct, intervalS)
	<< unset ("multiplot")
	);
}
