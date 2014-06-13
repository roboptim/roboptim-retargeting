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

#ifndef ROBOPTIM_RETARGETING_MORPHING_HH
# define ROBOPTIM_RETARGETING_MORPHING_HH
# include <map>
# include <string>
# include <vector>

# include <roboptim/core/function.hh> // for Eigen with the right flags.

# include <roboptim/retargeting/config.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Define a pair (marker, offset).
    struct ROBOPTIM_RETARGETING_DLLEXPORT MorphingDataMapping
    {
      std::string marker;
      Eigen::Vector3d offset;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /// \brief Morphing Data
    ///
    /// Morphing data gathers all the information required to fit a
    /// marker set to a virtual actor.
    ///
    /// In practice, it links bodies to markers.
    ///
    /// I.e. on body X, markers Y and Z are attached. A translation
    /// offset can also be specified for each marker (3d vector).
    ///
    /// The position of the marker Y attached to the body X in the
    /// Euclidian space is the position of body X plus a constant
    /// offset (specific to this particular pair body/marker).
    struct ROBOPTIM_RETARGETING_DLLEXPORT MorphingData
    {
      typedef std::vector<
	MorphingDataMapping,
	Eigen::aligned_allocator<MorphingDataMapping> >
      mappingData_t;

      typedef std::map<
	std::string,
	mappingData_t,
	std::less<std::string>,
	Eigen::aligned_allocator<
	  std::pair<
	    const std::string,
	    mappingData_t>
	  >
	>
      mapping_t;

      /// \brief Mapping from body names to markers (and offsets).
      mapping_t mapping;

      /// \brief Markers list
      std::vector<std::string> markers;

      /// \brief Get body on which a marker is attached.
      ///
      /// Throw a std::runtime_error is the marker cannot be found.
      ///
      /// \param[in] marker Marker name
      /// \return Link name
      ROBOPTIM_RETARGETING_DLLEXPORT const std::string&
	attachedBody (const std::string& marker) const;

      /// \brief Get offset from link and marker name.
      ///
      /// Throw a std::runtime_error if the link/marker is not found.
      ///
      /// \param[in] linkName link name
      /// \param[in] markerName marker name
      /// \return Position of the marker w.r.t link.
      ROBOPTIM_RETARGETING_DLLEXPORT Eigen::Vector3d
      offset (const std::string& linkName, const std::string& markerName) const;
    };

    /// \brief Load morphing data from YAML file.
    ///
    /// Load morphing data from YAML file.
    /// The file format definition is documented in:
    /// `share/roboptim/retargeting/data/human-to-hrp4c.morphing.yaml`
    ///
    /// Note that there is no consistency checks done at this level.
    /// The bodies and/or markers may or may not exist and are not
    /// linked to a particular marker set or robot model at this
    /// point.
    ///
    /// If an error occurs, this function throw a std::runtime_error
    /// exception or a subtype of this class.
    ///
    /// \param[in] YAML file path
    /// \return Morphing data.
    ROBOPTIM_RETARGETING_DLLEXPORT MorphingData
    loadMorphingData (const std::string& filename);

  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_MORPHING_HH
