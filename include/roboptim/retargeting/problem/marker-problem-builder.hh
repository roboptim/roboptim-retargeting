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
#ifndef ROBOPTIM_RETARGETING_PROBLEM_MARKER_PROBLEM_BUILDER_HH
# define ROBOPTIM_RETARGETING_PROBLEM_MARKER_PROBLEM_BUILDER_HH
# include <boost/shared_ptr.hpp>

# include <roboptim/retargeting/problem/problem-builder.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Marker optimization problem description.
    ///
    /// This structure contains the instruction to build the problem.
    /// I.e. which data should be loaded, what strategy should be
    /// chosen from resolution, etc.
    struct MarkerProblemOptions
    {
      /// \brief Frames before this one will not be considered
      int startFrame;

      /// \brief How many frames will be considered?
      ///
      /// -1 means all frames.
      int length;

      /// \brief Marker set filename.
      ///
      /// Any format supported by libmocap is acceptable.
      /// e.g. ".mars" Cortex file can be used
      std::string markerSet;

      /// \brief Markers trajectories.
      ///
      /// Markers trajectories which will be used as the initial input
      /// of the optimization problem.
      ///
      /// Any format supported by libmocap is acceptable.
      /// e.g. ".trc" Cortex file can be used.
      std::string markersTrajectory;

      /// \brief What type of trajectory should be used?
      ///
      /// Possible options are:
      /// - discrete
      /// - spline
      ///
      /// See roboptim-trajectory documentation for details.
      std::string trajectoryType;

      /// \brief Robot model to be used.
      ///
      /// The robot model is used to determine the segment length.
      std::string robotModel;

      /// \brief Solver plug-in name.
      std::string plugin;

      /// \brief Cost function name.
      std::string cost;

      /// \brief Constraints functions names.
      std::vector<std::string> constraints;

      /// \brief Final joint trajectory filename
      ///
      /// This file will be written at the end of the optimization
      /// process if it terminates successfully (or with warnings).
      std::string outputFile;
    };

    // Defined in function-factory.hh
    struct MarkerFunctionData;

    /// \brief Build marker problem data from problem description.
    ///
    /// The MarkerProblemData structure contains the real loaded data
    /// where the MarkerProblemOptions just describe the problem a set
    /// of strings (usually retrieved from the user).
    ///
    /// This function will load the motion capture data, robot model,
    /// etc.
    ///
    /// \note A failure at this stage is not fatal. Robot model for
    /// instance may be omitted if the segment length constraint is
    /// disabled. However, a fatal error however may occur when the
    /// constraints are allocated if a required data is missing from
    /// this structure.
    ///
    /// \param[out] data structure to contain the loaded data
    /// \param[in] options problem description
    inline void
    buildDataFromOptions (MarkerFunctionData& data,
			  const MarkerProblemOptions& options);

    /// \brief Build a marker retargeting problem.
    ///
    /// This class is in charge of building the optimization problem:
    /// the cost function, the constraints (function, intervals,
    /// scale) and will set the starting point of the optimization
    /// process.
    ///
    /// It will also load the data that which will be used by the
    /// constraints function.
    ///
    /// This class is parametrized by T, the problem type. By changing
    /// the problem type, sparse problems can also be solved.
    ///
    /// \tparam T problem type
    template <typename T>
    class MarkerProblemBuilder : public ProblemBuilder<T>
    {
    public:
      /// \brief Constructor and destructor
      /// \{

      /// \brief Constructor
      ///
      /// \warning options are kept as a const reference so this
      /// object lifespan much be longer than the one of this object.
      ///
      /// \param[in] options problem description
      MarkerProblemBuilder (const MarkerProblemOptions& options);
      ~MarkerProblemBuilder ();
      /// \}

      /// \brief Instantiate the problem and return it.
      ///
      /// \return shared pointer containing the newly created problem.
      void operator () (boost::shared_ptr<T>& problem,
			MarkerFunctionData& data);

    private:
      /// \brief Problem description.
      const MarkerProblemOptions& options_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

# include <roboptim/retargeting/problem/marker-problem-builder.hxx>
#endif //! ROBOPTIM_RETARGETING_PROBLEM_MARKER_PROBLEM_BUILDER_HH
