#include <fstream>
#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/acceleration-energy.hh"

namespace roboptim
{
  namespace retargeting
  {
    AccelerationEnergy::AccelerationEnergy
    (cnoid::MarkerIMeshPtr mesh) throw ()
      : roboptim::GenericNumericQuadraticFunction<EigenMatrixSparse>
	(matrix_t
	 (mesh->numFrames () * mesh->numActiveVertices () * 3,
	  mesh->numFrames () * mesh->numActiveVertices () * 3),
	 vector_t (mesh->numFrames () * mesh->numActiveVertices () * 3))
    {
      const double weight = 1e-6;
      const double dt = mesh->getTimeStep();
      const double k = weight / ((dt * dt) * (dt * dt));


      typedef Eigen::Triplet<double> triplet_t;

      // Fill the matrix first.
      std::vector<triplet_t> coefficients;
      for (int col = 0;
	   col < mesh->numFrames () * mesh->numActiveVertices () * 3;
	   ++col)
	coefficients.push_back (triplet_t (0, col, 0.));
      this->A ().setFromTriplets (coefficients.begin (), coefficients.end ());

      // Compute coefficients.
      for (int frameId = 0; frameId < mesh->numFrames (); ++frameId)
	{
	  for (int markerId = 0; markerId < mesh->numActiveVertices ();
	       ++markerId)
	    {
	      int prevOffset =
		(frameId - 1) * mesh->numActiveVertices () * 3 + (markerId * 3);
	      int currentOffset =
		frameId * mesh->numActiveVertices () * 3 + (markerId * 3);
	      int succOffset =
		(frameId + 1) * mesh->numActiveVertices () * 3 + (markerId * 3);

	      // V_{i-1}
	      if (prevOffset >= 0)
		{
		  this->A ().coeffRef (prevOffset + 0, currentOffset + 0) += 1. * k;
		  this->A ().coeffRef (prevOffset + 1, currentOffset + 1) += 1. * k;
		  this->A ().coeffRef (prevOffset + 2, currentOffset + 2) += 1. * k;
		}
	      // V_i
	      this->A ().coeffRef
		(currentOffset + 0, currentOffset + 0) += -2. * k;
	      this->A ().coeffRef
		(currentOffset + 1, currentOffset + 1) += -2. * k;
	      this->A ().coeffRef
		(currentOffset + 2, currentOffset + 2) += -2. * k;
	      // V_{i+1}
	      if (succOffset < this->A ().rows ())
		{
		  this->A ().coeffRef (succOffset + 0, currentOffset + 0) += 1. * k;
		  this->A ().coeffRef (succOffset + 1, currentOffset + 1) += 1. * k;
		  this->A ().coeffRef (succOffset + 2, currentOffset + 2) += 1. * k;
		}
	    }
	}

      this->A () = this->A ().transpose () * this->A ();

      this->A ().finalize ();
      this->b ().setZero ();

      // Print A in file (dense representation).
      {
	std::ofstream file ("/tmp/acceleration-A.dat");
	file.setf (std::ios::right);
	for (Eigen::MatrixXd::Index row = 0; row < A ().rows (); ++row)
	  {
	    for (Eigen::MatrixXd::Index col = 0; col < A ().cols (); ++col)
	      file << std::showpos << std::fixed << std::setprecision (8)
		   << A ().coeff (row, col) << " ";
	    file << "\n";
	  }
      }
      {
	std::ofstream file ("/tmp/acceleration-b.dat");
	for (Eigen::MatrixXd::Index elt = 0; elt < b ().size (); ++elt)
	  file << b ()[elt] << "\n";
      }
    }

    AccelerationEnergy::~AccelerationEnergy () throw ()
    {}
  } // end of namespace retargeting.
} // end of namespace roboptim.
