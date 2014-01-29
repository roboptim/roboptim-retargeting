#ifndef ROBOPTIM_RETARGETING_FUNCTION_LAPLACIAN_COORDINATE_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FUNCTION_LAPLACIAN_COORDINATE_CHOREONOID_HH
# include <stdexcept>
# include <roboptim/core/numeric-linear-function.hh>
# include <cnoid/BodyIMesh>

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
      (cnoid::BodyIMeshPtr mesh,
       int frameId,
       const vector_t& originalMarkerPosition) throw (std::runtime_error)
	: GenericNumericLinearFunction<T>
	  (matrix_t (mesh->numMarkers () * 3, mesh->numMarkers () * 3),
	   vector_t (mesh->numMarkers () * 3))
      {
	matrix_t& A = this->A ();
	vector_t& b = this->b ();

	// Compute A
	A.setIdentity ();
	const cnoid::BodyIMesh::Frame& neighborLists =
	  mesh->frame (frameId);
	for (int markerId = 0; markerId < mesh->numMarkers (); ++markerId)
	  {
	    const cnoid::BodyIMesh::NeighborList&
	      neighbors = neighborLists[markerId];
	    for (int l = 0; l < neighbors.size (); ++l)
	      {
		const int neighborId = neighbors[l];
		double weight =
		  (originalMarkerPosition.segment (markerId * 3, 3) -
		   originalMarkerPosition.segment (neighborId * 3, 3)).norm ();
		if (std::abs (weight) > 1e-8)
		  weight = 1. / weight;
		else
		  weight = 0.;

		// if i > j w(i,j) becomes w(j,i) as we will have i is
		// a neighbor of j and j is a neighbor of i, divide by
		// two the weight.
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
