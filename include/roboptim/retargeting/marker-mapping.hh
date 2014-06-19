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

#ifndef ROBOPTIM_RETARGETING_MARKER_MAPPING_HH
# define ROBOPTIM_RETARGETING_MARKER_MAPPING_HH
# include <string>
# include <utility>
# include <vector>

# include <boost/bimap.hpp>

# include <libmocap/marker-trajectory.hh>

# include <roboptim/core/function.hh> //FIXME: for Eigen

# include <roboptim/retargeting/config.hh>
# include <roboptim/retargeting/morphing.hh>
# include <roboptim/retargeting/utility.hh>

namespace roboptim
{
  namespace retargeting
  {
    ROBOPTIM_RETARGETING_PREDECLARE_CLASS (MarkerMapping);

    /// \brief Store mapping from marker name to its ID (position in
    ///        the optimization vector) and vice-versa.
    ///
    /// Marker order can be trivially obtained from a TRC file.
    ///
    /// However, when optimizing joints positions, no particular order
    /// is set.
    ///
    // In this case, an arbitrary order is determined. To
    /// ensure safe manipulation and ease error reporting, marker
    /// names are stored in objects instead of their ID. This object
    /// is passed along to be able to convert marker names to IDs and
    /// vice-versa.
    class ROBOPTIM_RETARGETING_DLLEXPORT MarkerMapping
    {
    public:
      typedef boost::bimap<std::string, std::size_t> map_t;
      typedef map_t::iterator map_iterator_t;

      typedef map_t::left_const_iterator left_const_iterator_t;
      typedef map_t::right_const_iterator right_const_iterator_t;

      std::size_t markerId (const std::string) const;
      Eigen::VectorXd::Index markerIdEigen (const std::string) const;

      std::string markerName (std::size_t) const;
      std::string markerNameEigen (Eigen::VectorXd::Index) const;

      std::size_t numMarkers () const;
      Eigen::VectorXd::Index numMarkersEigen () const;

      template <typename Derived>
      Eigen::VectorBlock<const Derived, 3>
      marker (const Eigen::MatrixBase<Derived>& x, const std::string markerName) const
      {
	typename Eigen::MatrixBase<Derived>::Index id = this->markerIdEigen (markerName);
	typename Eigen::MatrixBase<Derived>::Index start = id *3;
	return x.template segment<3> (start);
      }

      ROBOPTIM_RETARGETING_ACCESSOR (mapping, map_t);

      std::ostream& print (std::ostream& o) const;
    private:
      map_t mapping_;
    };

    /// \brief Override the operator<< to display mapping objects.
    ///
    /// Like many other RobOptim classes, real display is done by the
    /// MarkerMapping::print method.
    ROBOPTIM_RETARGETING_DLLEXPORT std::ostream&
    operator<< (std::ostream& o, const MarkerMapping& mapping);

    /// \brief Build marker mapping from markers trajectory.
    ///
    /// A mapping between markers names and their ID can be created
    /// using markers trajectory. libmocap provides all the information to do so.
    ///
    /// \param[in] markerTrajectory libmocap markers trajectory
    /// \return return a shared pointer to the newly instantiated object
    ROBOPTIM_RETARGETING_DLLEXPORT MarkerMappingShPtr
    buildMarkerMappingFromMotion (const libmocap::MarkerTrajectory& markerTrajectory);

    /// \brief Build marker mapping from a morphing object.
    ///
    /// When markers trajectory is not available (i.e. joint based
    /// optimization), marker mapping can alternatively be built from
    /// a morphing object.
    ///
    /// \warning there is no guarantee that the order will be the same
    ///		 that when using the #buildMarkerMappingFromMotion
    ///		 function.
    ///
    /// \param [in] morphing morphing data
    /// \return return a shared pointer to the newly instantiated object
    ROBOPTIM_RETARGETING_DLLEXPORT MarkerMappingShPtr
    buildMarkerMappingFromMorphing (const MorphingData& morphing);
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_INTERACTION_MESH_HH

