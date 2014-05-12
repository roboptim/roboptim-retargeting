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
#ifndef ROBOPTIM_RETARGETING_PROBLEM_MARKER_TO_JOINT_PROBLEM_BUILDER_HH
# define ROBOPTIM_RETARGETING_PROBLEM_MARKER_TO_JOINT_PROBLEM_BUILDER_HH
# include <string>
# include <vector>

# include <boost/shared_ptr.hpp>

# include <roboptim/retargeting/problem/problem-builder.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Joint optimization problem description.
    ///
    /// This structure contains the instruction to build the problem.
    /// I.e. which data should be loaded, what strategy should be
    /// chosen from resolution, etc.
    struct MarkerToJointProblemOptions
    {
      std::string markerSet;
      std::string markersTrajectory;

      std::string trajectoryType;

      std::string outputFile;

      std::string robotModel;

      std::string cost;

      std::string plugin;

      /// \brief Disabled joints
      ///
      /// Disabled DOFs will be excluded from the optimization problem
      /// and hence reduce the overall size of the problem.
      std::vector<std::string> disabledJoints;


      Function::vector_t::Index frameId;
    };

    // Defined in function-factory.hh
    struct MarkerToJointFunctionData;

    /// \brief Build joint problem data from problem description.
    ///
    /// The JointProblemData structure contains the loaded data
    /// whereas the JointProblemOptions just describe the problem a
    /// set of strings (usually retrieved from the user).
    ///
    /// This function will load the joints trajectories, robot model,
    /// etc.
    ///
    /// \note A failure at this stage is not fatal. Some data loading
    /// may be skipped with a warning message. However, a fatal error
    /// however may occur when the constraints are allocated if a
    /// required data is missing from this structure.
    ///
    /// \param[out] data structure to contain the loaded data
    /// \param[in] options problem description
    void
    buildJointDataFromOptions (MarkerToJointFunctionData& data,
			       const MarkerToJointProblemOptions& options);

    /// \brief Build a joint retargeting problem.
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
    class MarkerToJointProblemBuilder : public ProblemBuilder<T>
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
      MarkerToJointProblemBuilder (const MarkerToJointProblemOptions& options);
      ~MarkerToJointProblemBuilder ();
      /// \}

      /// \brief Instantiate the problem and return it.
      ///
      /// \return shared pointer containing the newly created problem.
      void operator () (boost::shared_ptr<T>& problem,
			MarkerToJointFunctionData& data);

    private:
      /// \brief Problem description.
      const MarkerToJointProblemOptions& options_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

# include <roboptim/retargeting/problem/marker-to-joint-problem-builder.hxx>
#endif //! ROBOPTIM_RETARGETING_PROBLEM_MARKER_TO_JOINT_PROBLEM_BUILDER_HH
