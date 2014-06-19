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

#include <fstream>
#include <string>

#include <boost/format.hpp>
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
#include <roboptim/core/util.hh>

#include <roboptim/trajectory/trajectory.hh>

#include <roboptim/retargeting/exception.hh>
#include <roboptim/retargeting/io/choreonoid-body-motion.hh>
#include <roboptim/retargeting/problem/marker-to-joint-problem-builder.hh>

#include "path.hh"

static bool parseOptions
(roboptim::retargeting::MarkerToJointProblemOptions& options,
 int argc, const char* argv[])
{
  namespace po = boost::program_options;
  po::options_description desc ("Options");
  desc.add_options ()
    ("help,h", "Print help messages")
    ("marker-trajectory,m",
     po::value<std::string>
     (&options.markersTrajectory)->required (),
     "input markers trajectory used during Motion Capture"
     " (trc or any other format supported by libmocap)")
    ("output-file,o",
     po::value<std::string>
     (&options.outputFile)->required (),
     "output marker trajectory (Choreonoid YAML file)")
    ("trajectory-type,t",
     po::value<std::string>
     (&options.trajectoryType)->default_value ("discrete"),
     "Trajectory type (discrete)")

    ("marker-set,s",
     po::value<std::string> (&options.markerSet)->required (),
     "Marker Set used during Motion Capture"
     " (mars or any other format supported by libmocap)")

    ("robot-model,r",
     po::value<std::string> (&options.robotModel)->required (),
     "Robot Model (Choreonoid YAML file)")

    ("morphing,M",
     po::value<std::string> (&options.morphing)->required (),
     "Morphing data (YAML file)")

    ("constraint,C",
     po::value<std::vector<std::string> > (&options.constraints),
     "Which constraints should be used?")

    ("disable-joint,d",
     po::value<std::vector<std::string> > (&options.disabledJoints),
     "Exclude a joint from the optimization process")

    ("cost,c",
     po::value<std::string> (&options.cost)->default_value ("null"),
     "What cost function should be used?")
    ("plugin,p",
     po::value<std::string> (&options.plugin)->default_value ("cfsqp"),
     "RobOptim plug-in to be used")

    ("start-frame,S",
     po::value<int> (&options.startFrame)->default_value (0),
     "From what frame should we start converting?")
    ("length,l",
     po::value<int> (&options.length)->default_value (0),
     "How many frames? (0 means all frames, "
     "negative number means exclude N last frames)")

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

      std::cout << "Available functions:\n";

      std::vector<std::string> functions =
	roboptim::retargeting::MarkerToJointFunctionFactory::listFunctions ();
      std::vector<std::string>::const_iterator it;
      for (it = functions.begin (); it != functions.end (); ++it)
	std::cout << "\t - " << *it << "\n";

      return false;
    }

  po::notify (vm);

  roboptim::retargeting::resolvePath (options.markerSet);
  roboptim::retargeting::resolvePath (options.markersTrajectory);
  roboptim::retargeting::resolvePath (options.robotModel);
  roboptim::retargeting::resolvePath (options.morphing);
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
  data.initialized = false;
  options.frameId = 0;
  builder (problem, data);

  roboptim::Function::vector_t::Index nFrames =
    static_cast<roboptim::Function::vector_t::Index>
    (data.markersTrajectory.numFrames ());

  if (options.length == 0)
    options.length = static_cast<int> (nFrames);
  else if (options.length < 0)
    options.length = static_cast<int> (nFrames) - options.length;
  if (options.length <= 0)
    throw std::runtime_error
      ("less than 0 frame or zero frame have been selected");

  std::ostream& o = std::cout;

  for (options.frameId = options.startFrame;
       options.frameId < options.length; ++options.frameId)
    {
      o << "╔═════════════════════╤═════════════════╗" << roboptim::iendl
	<< (boost::format
	    ("║ Optimizing Frame... │ %-6d / %-6d ║")
	    % options.frameId
	    % (options.length - 1)).str () << roboptim::iendl
	<< "╚═════════════════════╧═════════════════╝" << roboptim::iendl
	;

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

      std::cout << solver << roboptim::resetindent << roboptim::iendl;

      const solver_t::result_t& result = solver.minimum ();

      roboptim::Function::vector_t parameters =
	data.outputTrajectoryReduced->parameters ();

      roboptim::Function::vector_t::Index length = data.nDofsFiltered ();
      roboptim::Function::vector_t::Index start =
	static_cast<roboptim::Function::vector_t::Index>
	(options.frameId * length);


      if (result.which () == solver_t::SOLVER_VALUE_WARNINGS)
	{
	  std::cout << "Optimization finished. Warnings have been issued\n";
	  roboptim::ResultWithWarnings result_ =
	    boost::get<roboptim::ResultWithWarnings> (result);
	  std::cerr << result << roboptim::iendl;

	  parameters.segment (start, length) = result_.x;
	}
      else if (result.which () == solver_t::SOLVER_VALUE)
	{
	  std::cout << "Optimization finished successfully.\n";
	  roboptim::Result result_ =
	    boost::get<roboptim::Result> (result);
	  std::cerr << result << roboptim::iendl;

	  parameters.segment (start, length) = result_.x;
	}
      else
	{
	  throw std::runtime_error ("Optimization failed");
	}

      data.outputTrajectoryReduced->setParameters (parameters);

      // Normalize angles in the trajectory (except base position)
      for (roboptim::Function::size_type dofId = 0;
	   dofId < data.nDofsFiltered () - 3; ++dofId)
	data.outputTrajectoryReduced->normalizeAngles (3 + dofId);
    }

  // Re-expend trajectory.
  roboptim::Function::vector_t finalTrajectoryParameters =
    data.outputTrajectory->parameters ();
  for (roboptim::Function::vector_t::Index frameId = 0;
       frameId < nFrames; ++frameId)
    {
      roboptim::Function::vector_t::Index jointIdReduced = 0;
      for (roboptim::Function::vector_t::Index jointId = 0;
	   jointId < data.nDofsFull (); ++jointId)
        {
          if (data.disabledJointsConfiguration
	      [static_cast<std::size_t> (jointId)])
	    finalTrajectoryParameters[frameId * data.nDofsFull () + jointId] =
	      *(data.disabledJointsConfiguration)
	      [static_cast<std::size_t> (jointId)];
	  else
	    {
	      finalTrajectoryParameters[frameId * data.nDofsFull () + jointId] =
		data.outputTrajectoryReduced->parameters ()
		[frameId * data.nDofsFiltered () + jointIdReduced++];
	    }
        }
    }
  data.outputTrajectory->setParameters (finalTrajectoryParameters);
  roboptim::retargeting::writeBodyMotion
    (options.outputFile, data.outputTrajectory);
  return 0;
}


int main (int argc, const char* argv[])
{
  try
    {
      return safeMain (argc, argv);
    }
  catch (const roboptim::retargeting::Exception& e)
    {
      std::cerr << e << roboptim::iendl;
      return 1;
    }
  catch (const std::exception& e)
    {
      std::cerr << e.what () << roboptim::iendl;
      return 1;
    }
}
