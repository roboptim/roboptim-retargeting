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
#include <algorithm>

#include <boost/format.hpp>
#include <boost/make_shared.hpp>

#include <roboptim/core/indent.hh>

#include <roboptim/retargeting/exception.hh>
#include <roboptim/retargeting/marker-mapping.hh>
#include <roboptim/retargeting/utility.hh>

namespace roboptim
{
  namespace retargeting
  {
    std::size_t MarkerMapping::markerId (const std::string markerName) const
    {
      left_const_iterator_t it = mapping_.left.find (markerName);
      if (it == mapping_.left.end ())
	throw MarkerNotFound
	  (markerName, __FILE__, __LINE__, ROBOPTIM_RETARGETING_FUNCTION);
      return it->second;
    }

    Eigen::VectorXd::Index
    MarkerMapping::markerIdEigen (const std::string markerName) const
    {
      return static_cast<Eigen::VectorXd::Index> (this->markerId (markerName));
    }

    std::size_t MarkerMapping::numMarkers () const
    {
      return mapping_.size ();
    }

    Eigen::VectorXd::Index MarkerMapping::numMarkersEigen () const
    {
      return static_cast<Eigen::VectorXd::Index> (this->numMarkers ());
    }

    std::string
    MarkerMapping::markerName (std::size_t id) const
    {
      right_const_iterator_t it = mapping_.right.find (id);
      if (it == mapping_.right.end ())
	throw MarkerNotFound
	  ((boost::format ("%d") % id).str (),
	   __FILE__, __LINE__, ROBOPTIM_RETARGETING_FUNCTION);
      return it->second;
    }

    std::string
    MarkerMapping::markerNameEigen (Eigen::VectorXd::Index id) const
    {
      return markerName (static_cast<std::size_t> (id));
    }

    std::ostream&
    MarkerMapping::print (std::ostream& o) const
    {
      left_const_iterator_t it;
      for (it = mapping_.left.begin (); it != mapping_.left.end (); ++it)
	o << "- " << it->first << ": " << it->second << iendl;
      return o;
    }

    MarkerMappingShPtr
    buildMarkerMappingFromMotion (const libmocap::MarkerTrajectory& markerTrajectory)
    {
      MarkerMappingShPtr result = boost::make_shared<MarkerMapping> ();

      std::vector<std::string>::const_iterator it;
      std::size_t id = 0;

      for (it = markerTrajectory.markers ().begin ();
	   it != markerTrajectory.markers ().end (); ++it)
	safeGet (result).mapping ().insert
	  (MarkerMapping::map_t::value_type (*it, id++));

      return result;
    }

    MarkerMappingShPtr
    buildMarkerMappingFromMorphing (const MorphingData& morphing)
    {
      MarkerMappingShPtr result = boost::make_shared<MarkerMapping> ();

      std::vector<std::string>::const_iterator it;
      std::size_t id = 0;

      for (it = morphing.markers.begin ();
	   it != morphing.markers.end (); ++it)
	safeGet (result).mapping ().insert
	  (MarkerMapping::map_t::value_type (*it, id++));

      return result;
    }

    std::ostream&
    operator<< (std::ostream& o, const MarkerMapping& mapping)
    {
      return mapping.print (o);
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.
