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
    struct MarkerProblemOptions
    {
      std::string markerSet;
      std::string markersTrajectory;
      std::string robotModel;
      std::string plugin;

      std::string cost;
      std::vector<std::string> constraints;
    };


    /// \brief Build a marker retargeting problem.
    template <typename T>
    class MarkerProblemBuilder : public ProblemBuilder<T>
    {
    public:
      MarkerProblemBuilder (const MarkerProblemOptions& options);
      ~MarkerProblemBuilder ();
      boost::shared_ptr<T> operator () ();

    private:
      const MarkerProblemOptions& options_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

# include <roboptim/retargeting/problem/marker-problem-builder.hxx>
#endif //! ROBOPTIM_RETARGETING_PROBLEM_MARKER_PROBLEM_BUILDER_HH
