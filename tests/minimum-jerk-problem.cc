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

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include "tests-config.h"

#include <roboptim/core/filter/derivative.hh>
#include <roboptim/core/visualization/gnuplot.hh>
#include <roboptim/core/visualization/gnuplot-commands.hh>
#include <roboptim/core/visualization/gnuplot-function.hh>
#include <roboptim/trajectory/state-function.hh>
#include <roboptim/trajectory/trajectory-sum-cost.hh>
#include <roboptim/trajectory/vector-interpolation.hh>
#include <roboptim/retargeting/function/minimum-jerk-trajectory.hh>

using namespace roboptim;
using namespace roboptim::visualization;
using namespace roboptim::retargeting;

using boost::test_tools::output_test_stream;

boost::array<double, 44> standardPose = {{
  0, 0, -25, 50, -25, 0, 0,
  0, 0, -25, 50, -25, 0, 0,
  0, 0, 0,
  0, 0, 0,
  -1.0, 1.0, 0, 0, 0, -1.0, 1.0, -1.0,
  5, -10, 0, -15, 0,  10,  1.0, 0,
  5,  10, 0, -15, 0, -10, -1.0, 0
  }};

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

  explicit CostReferenceTrajectory (boost::shared_ptr<Trajectory<3> > referenceTrajectory,
				    size_type dofId,
				    value_type dt)
    throw ()
    : GenericDifferentiableFunction<T>
      (referenceTrajectory->parameters ().size (), 1, "CostReferenceTrajectory"),
      referenceTrajectory_ (referenceTrajectory),
      vectorInterpolation_
      (boost::make_shared<VectorInterpolation>
       (vector_t::Zero (referenceTrajectory->parameters ().size ()),
	referenceTrajectory->outputSize (), dt)),
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
	  * vectorInterpolation_->variationStateWrtParam (t, 1);
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


BOOST_AUTO_TEST_CASE (simple)
{
  typedef MinimumJerkTrajectory<EigenMatrixDense>::vector_t vector_t;
  typedef MinimumJerkTrajectory<EigenMatrixDense>::value_type value_type;
  typedef MinimumJerkTrajectory<EigenMatrixDense>::size_type size_type;

  std::size_t nFrames = 100.;
  value_type dt = 0.005;
  value_type tmin = 0.;
  value_type tmax = dt * static_cast<value_type> (nFrames);
  value_type init = 0.;
  value_type goal = 1.;
  // the id of the dof we move.
  std::size_t dofId = 0;
  std::size_t nDofs = standardPose.size ();

  configureLog4cxx ();

  // compute the initial trajectory (whole body)
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

  for (std::size_t frameId = 0; frameId < nFrames; ++frameId)
    {
      for (std::size_t dof = 0; dof < nDofs; ++dof)
	initialTrajectory[frameId * nDofs + dof] = standardPose[dof];

      initialTrajectory[frameId * nDofs + dofId] =
	(*minimumJerkTrajectory)
	(static_cast<value_type> (frameId) * dt)[0];
    }

  MinimumJerkTrajectory<EigenMatrixDense>::discreteInterval_t
    intervalS (tmin, tmax, 0.01);

  boost::shared_ptr<VectorInterpolation >
    initialTrajectoryFct =
    vectorInterpolation
    (initialTrajectory, static_cast<size_type> (nDofs), dt);

  boost::shared_ptr<CostReferenceTrajectory<EigenMatrixDense> >
    cost = boost::make_shared<CostReferenceTrajectory<EigenMatrixDense> >
    (initialTrajectoryFct, dofId, dt);

  // Create problem.

  // Solve problem.


  // Display initial and final trajectory.
  using namespace roboptim::visualization::gnuplot;
  Gnuplot gnuplot = Gnuplot::make_interactive_gnuplot ();
  std::cout
    << (gnuplot
	<< set ("multiplot layout 2, 1")
	<< plot (*minimumJerkTrajectory, intervalS)
	<< plot (*initialTrajectoryFct, intervalS)
	<< unset ("multiplot")
	);
}
