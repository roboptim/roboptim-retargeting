// Copyright (C) 2014 by Thomas Moulard, AIST, CNRS.
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

#include <string>

#include <boost/program_options.hpp>

#include <cnoid/Body>
#include <cnoid/BodyLoader>

#include <libmocap/marker-set.hh>
#include <libmocap/marker-set-factory.hh>
#include <libmocap/marker-trajectory.hh>
#include <libmocap/marker-trajectory-factory.hh>

#include <roboptim/core/solver.hh>
#include <roboptim/core/solver-factory.hh>
#include <roboptim/core/result.hh>
#include <roboptim/core/result-with-warnings.hh>

#include <roboptim/trajectory/trajectory.hh>

#include <roboptim/retargeting/problem/marker-to-joint-problem-builder.hh>

static bool parseOptions
(roboptim::retargeting::MarkerToJointProblemOptions& options,
 int argc, const char* argv[])
{
  namespace po = boost::program_options;
  po::options_description desc ("Options");
  desc.add_options ()
    ("help,h", "Print help messages")
    ("markers-trajectory,m",
     po::value<std::string>
     (&options.markersTrajectory)->required (),
     "input markers trajectory used during Motion Capture"
     " (trc or any other format supported by libmocap)")
    ("output-file,o",
     po::value<std::string>
     (&options.outputFile)->required (),
     "output marker trajectory (Choreonoid YAML file)")

    ("marker-set,s",
     po::value<std::string> (&options.markerSet)->required (),
     "Marker Set used during Motion Capture"
     " (mars or any other format supported by libmocap)")

    ("robot-model,r",
     po::value<std::string> (&options.robotModel)->required (),
     "Robot Model (Choreonoid YAML file)")
    ;

  po::variables_map vm;
  po::store
    (po::command_line_parser (argc, argv)
     .options (desc)
     .run (),
     vm);

  if (vm.count ("help"))
    {
      std::cout << desc << "\n";
      return false;
    }

  po::notify (vm);

  return true;
}


int safeMain (int argc, const char* argv[])
{
  typedef roboptim::retargeting::denseProblem_t problem_t;
  typedef roboptim::Solver<
    roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense>,
    boost::mpl::vector<
      roboptim::GenericLinearFunction<roboptim::EigenMatrixDense>,
      roboptim::GenericDifferentiableFunction<roboptim::EigenMatrixDense>
      >
    >
    solver_t;

  roboptim::retargeting::MarkerToJointProblemOptions options;

  if (!parseOptions (options, argc, argv))
    return 0;

  // Build problem.
  roboptim::retargeting::MarkerToJointProblemBuilder<problem_t>
    builder (options);

  boost::shared_ptr<problem_t> problem;
  roboptim::retargeting::MarkerToJointFunctionData data;
  builder (problem, data);

  if (!problem)
    throw std::runtime_error ("failed to build problem");

  roboptim::SolverFactory<solver_t>
    factory (options.plugin, *problem);
  solver_t& solver = factory ();

  // Set solver parameters.
  solver.parameters ()["max-iterations"].value = 1000;

  solver.parameters ()["ipopt.output_file"].value =
    "/tmp/ipopt.log";
  solver.parameters ()["ipopt.print_level"].value = 5;
  solver.parameters ()["ipopt.expect_infeasible_problem"].value = "no";
  solver.parameters ()["ipopt.nlp_scaling_method"].value = "none";
  solver.parameters ()["ipopt.tol"].value = 1e-3;
  solver.parameters ()["ipopt.dual_inf_tol"].value = 1.;
  solver.parameters ()["ipopt.constr_viol_tol"].value = 1e-3;

  // first-order
  solver.parameters ()["ipopt.derivative_test"].value = "first-order";
  solver.parameters ()["nag.verify-level"].value = 0;

  std::cout << solver << std::endl;

  const solver_t::result_t& result = solver.minimum ();

  if (result.which () == solver_t::SOLVER_VALUE_WARNINGS)
    {
      std::cout << "Optimization finished. Warnings have been issued\n";
      roboptim::ResultWithWarnings result_ =
        boost::get<roboptim::ResultWithWarnings> (result);
      std::cerr << result << std::endl;
    }
  else if (result.which () == solver_t::SOLVER_VALUE)
    {
      std::cout << "Optimization finished successfully.\n";
      roboptim::Result result_ =
        boost::get<roboptim::Result> (result);
      std::cerr << result << std::endl;
    }
  else
    {
      throw std::runtime_error ("Optimization failed");
    }

  //writeBodyMotion (options.outputFile, finalTrajectory);
  return 0;
}


int main (int argc, const char* argv[])
{
  try
    {
      return safeMain (argc, argv);
    }
  catch (const std::exception& e)
    {
      std::cerr << e.what () << std::endl;
      return 1;
    }
}
