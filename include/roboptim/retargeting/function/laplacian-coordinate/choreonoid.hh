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

#ifndef ROBOPTIM_RETARGETING_FUNCTION_LAPLACIAN_COORDINATE_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FUNCTION_LAPLACIAN_COORDINATE_CHOREONOID_HH
# include <stdexcept>

# include <roboptim/core/numeric-linear-function.hh>
# include <roboptim/retargeting/interaction-mesh.hh>
# include <roboptim/retargeting/morphing.hh>

# include <roboptim/retargeting/marker-mapping.hh>
# include <roboptim/retargeting/interaction-mesh.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Laplacian Coordinate of all markers of one frame.
    ///
    /// for N markers, respectively at position
    /// (x0, y0, z0), ..., (xN, yN, zN)
    ///
    /// Input:
    ///  x = [ x0, y0, z0, ... xN, yN, zN ]
    ///
    /// Output:
    ///  f(x) = [ x0, y0, z0, ... xN, yN, zN ]
    ///
    /// \tparam T Function traits type
    template <typename T>
    class LaplacianCoordinateChoreonoid
      : public GenericNumericLinearFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericNumericLinearFunction<T>);

      explicit LaplacianCoordinateChoreonoid
      (MarkerMappingShPtr markerMapping,
       InteractionMeshShPtr mesh,
       std::size_t frameId,
       const vector_t& originalMarkerPosition)
	: GenericNumericLinearFunction<T>
	  (matrix_t (safeGet (markerMapping).numMarkersEigen () * 3,
		     safeGet (markerMapping).numMarkersEigen () * 3),
	   vector_t (safeGet (markerMapping).numMarkersEigen () * 3))
      {
	ROBOPTIM_RETARGETING_PRECONDITION (!!markerMapping);
	ROBOPTIM_RETARGETING_PRECONDITION (!!mesh);

	matrix_t& A = this->A ();
	vector_t& b = this->b ();

	// Compute A
	A.setIdentity ();
	const InteractionMesh::neighborsMap_t& neighborsMap =
	  mesh->neighbors (frameId);
	for (InteractionMesh::neighborsMap_t::const_iterator itMarker = neighborsMap.begin ();
	     itMarker != neighborsMap.end (); ++itMarker)
	  {
	    for (InteractionMesh::neighbors_t::const_iterator itNeighbor = itMarker->second.begin ();
		 itNeighbor != itMarker->second.end (); ++itNeighbor)
	      {
		double weight =
		  (safeGet (markerMapping).marker (originalMarkerPosition, itMarker->first) -
		   safeGet (markerMapping).marker (originalMarkerPosition, *itNeighbor)).norm ();
		if (std::abs (weight) > 1e-8)
		  weight = 1. / weight;
		else
		  weight = 0.;

		// if i > j w(i,j) becomes w(j,i) as we will have i is
		// a neighbor of j and j is a neighbor of i, divide by
		// two the weight.
		size_type markerId = safeGet (markerMapping).markerIdEigen (itMarker->first);
		size_type neighborId = safeGet (markerMapping).markerIdEigen (*itNeighbor);
		A (std::min (markerId, neighborId),
		   std::max (markerId, neighborId)) -= weight / 2.;
	      }
	  }

	// Compute b.
	b.setZero ();
      }
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_LAPLACIAN_COORDINATE_CHOREONOID_HH
