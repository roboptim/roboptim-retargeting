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

#ifndef ROBOPTIM_RETARGETING_INTERACTION_MESH_HH
# define ROBOPTIM_RETARGETING_INTERACTION_MESH_HH
# include <map>
# include <set>
# include <string>
# include <utility>
# include <vector>

# include <roboptim/retargeting/config.hh>
# include <roboptim/retargeting/marker-mapping.hh>
# include <roboptim/retargeting/utility.hh>

namespace roboptim
{
  namespace retargeting
  {
    ROBOPTIM_RETARGETING_PREDECLARE_CLASS (InteractionMesh);

    class ROBOPTIM_RETARGETING_DLLEXPORT InteractionMesh
    {
    public:
      /// \brief List of neighbors.
      typedef std::set<std::string>  neighbors_t;

      /// \brief Map a marker to its neighbors.
      typedef std::map<std::string, neighbors_t> neighborsMap_t;

      const neighborsMap_t& neighbors (std::size_t frameId) const;

      neighborsMap_t& neighbors (std::size_t frameId);

      static InteractionMeshShPtr
      buildInteractionMeshFromMarkerMotion
      (const TrajectoryShPtr trajectory, MarkerMappingShPtr markerMapping);

      std::ostream& print (std::ostream&) const;
    private:
      /// \brief Map each marker to its neighbors for each frame.
      std::vector<neighborsMap_t> neighbors_;
    };

    ROBOPTIM_RETARGETING_DLLEXPORT std::ostream&
    operator<< (std::ostream&, const InteractionMesh&);

    /// \brief Build an interaction mesh from marker motion and marker
    ///        mapping.
    ///
    /// \param[in] trajectory marker trajectory
    /// \param[in] markerMapping marker mapping
    /// \return Interaction Mesh
    ROBOPTIM_RETARGETING_DLLEXPORT InteractionMeshShPtr
    buildInteractionMeshFromMarkerMotion
    (const TrajectoryShPtr trajectory,
     MarkerMappingShPtr markerMapping);

    /// \brief Build an interaction mesh from joint motion.
    ///
    /// \param[in] trajectory joint trajectory
    /// \return Interaction Mesh
    ROBOPTIM_RETARGETING_DLLEXPORT InteractionMeshShPtr
    buildInteractionMeshFromMarkerMotion (const TrajectoryShPtr trajectory);
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_INTERACTION_MESH_HH
