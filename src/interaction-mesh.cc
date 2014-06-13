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
#include <boost/make_shared.hpp>

#include <roboptim/retargeting/interaction-mesh.hh>
#include <roboptim/retargeting/utility.hh>

#include <tetgen.h>

namespace roboptim
{
  namespace retargeting
  {
    const InteractionMesh::neighborsMap_t&
    InteractionMesh::neighbors (std::size_t frameId) const
    {
      ROBOPTIM_RETARGETING_PRECONDITION (frameId < neighbors_.size ());
      return neighbors_[frameId];
    }

    InteractionMesh::neighborsMap_t&
    InteractionMesh::neighbors (std::size_t frameId)
    {
      ROBOPTIM_RETARGETING_PRECONDITION (frameId < neighbors_.size ());
      return neighbors_[frameId];
    }

    std::ostream&
    InteractionMesh::print (std::ostream& o) const
    {
      std::vector<neighborsMap_t>::const_iterator itFrame;
      for (itFrame = neighbors_.begin ();
	   itFrame != neighbors_.end (); ++itFrame)
	{
	  o << "* Frame:" << incindent << iendl;

	  if (itFrame->begin () == itFrame->end ())
	    o << "No frame" << iendl;
	  else
	    {
	      neighborsMap_t::const_iterator itMap;
	      for (itMap = itFrame->begin (); itMap != itFrame->end (); ++itMap)
		{
		  o << "- " << itMap->first << incindent << iendl;
		  if (itMap->second.begin () == itMap->second.end ())
		    o << "No neighbor" << iendl;
		  else
		    {
		      neighbors_t::const_iterator itNeighbor;
		      for (itNeighbor = itMap->second.begin ();
			   itNeighbor != itMap->second.end (); ++itNeighbor)
			o << "+ " << *itNeighbor << iendl;
		    }
		  o << decindent << iendl;
		}
	    }
	  o << decindent << iendl;

	  // Avoid flooding the output, later a better solution should
	  // be found.
	  if (itFrame - neighbors_.begin () >= 2)
	    {
	      o << "[output too long, truncated]" << iendl;
	      return o;
	    }
	}
      return o;
    }

    std::ostream&
    operator<< (std::ostream& o, const InteractionMesh& mesh)
    {
      return mesh.print (o);
    }

    template <typename Derived>
    InteractionMesh::neighborsMap_t
    buildInteractionMeshOneFrame
    (const Eigen::MatrixBase<Derived>& markerPositions,
     MarkerMappingShPtr markerMapping)
    {
      InteractionMesh::neighborsMap_t result;
      int dim = 3;
      int numpoints = static_cast<int> (markerPositions.size ()) / dim;

      tetgenio tetgenInput;
      tetgenio tetgenOutput;

      tetgenInput.numberofpoints = numpoints;
      tetgenInput.pointlist = new REAL[markerPositions.size ()];

      Eigen::Map<Eigen::Matrix<REAL, 1, Eigen::Dynamic> >
	input (tetgenInput.pointlist, markerPositions.size ());
      input = markerPositions;

      char switches[] = "zQ";
      try
	{
	  tetrahedralize (switches, &tetgenInput, &tetgenOutput);
	}
      catch (...)
	{
	  //FIXME: no neighbors for problematic points.
	  return result;
	}

      const int n = tetgenOutput.numberoftetrahedra;
      const int m = tetgenOutput.numberofcorners;

      for(int i = 0; i < n; ++i)
	{
	  for(int j = 0; j < m-1; ++j)
	    {
            for(int k = j + 1; k < m; ++k)
	      {
                int index0 = tetgenOutput.tetrahedronlist[i * m + j];
                int index1 = tetgenOutput.tetrahedronlist[i * m + k];

                if (index0 > index1)
		  std::swap (index0, index1);

		std::string markerName0 = safeGet (markerMapping).markerName
		  (static_cast<std::size_t> (index0));
		std::string markerName1 = safeGet (markerMapping).markerName
		  (static_cast<std::size_t> (index1));

		result[markerName0].insert (markerName1);
		result[markerName1].insert (markerName0);
	      }
	    }
	}
      return result;
    }

    InteractionMeshShPtr
    InteractionMesh::buildInteractionMeshFromMarkerMotion
    (const TrajectoryShPtr trajectory,
     MarkerMappingShPtr markerMapping)
    {
      InteractionMeshShPtr result = boost::make_shared<InteractionMesh> ();

      std::size_t nDiscretizationPoints =
	numberOfDiscretizationPoints (trajectory);
      safeGet(result).neighbors_.resize (nDiscretizationPoints);

      for (std::size_t p = 0; p < nDiscretizationPoints; ++p)
	{
	  StableTimePoint t = p / nDiscretizationPoints * tMax;
	  safeGet (result).neighbors (p) =
	    buildInteractionMeshOneFrame
	    (safeGet (trajectory) (t), markerMapping);
	}

      return result;
    }

    InteractionMeshShPtr
    buildInteractionMeshFromMarkerMotion
    (const TrajectoryShPtr trajectory, MarkerMappingShPtr markerMapping)
    {
      return InteractionMesh::buildInteractionMeshFromMarkerMotion
	(trajectory, markerMapping);
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.

