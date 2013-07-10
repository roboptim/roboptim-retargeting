#include <fstream>
#include <boost/format.hpp>
#include <roboptim/core/finite-difference-gradient.hh>
#include "roboptim/retargeting/position.hh"

namespace roboptim
{
  namespace retargeting
  {
    Position::Position
    (cnoid::MarkerIMeshPtr mesh,
     int motionIndex,
     int localMarkerIndex,
     const cnoid::Vector3& pos,
     bool isRelative) throw ()
      : roboptim::GenericNumericLinearFunction<EigenMatrixSparse>
	(matrix_t
	 (3 * mesh->numFrames (),
	  mesh->numFrames() * mesh->numActiveVertices() * 3),
	 vector_t
	 (3 * mesh->numFrames ()))
    {
      int activeVertexIndex =
	mesh->localToActiveIndex (motionIndex, localMarkerIndex);
      cnoid::MarkerMotionPtr motion = mesh->motion (motionIndex);

      A ().reserve (3 * mesh->numFrames ());
      for (int frameId = 0; frameId < mesh->numFrames (); ++frameId)
	{
	  const int row = 0;
	  const int col = activeVertexIndex * 3;

	  for(int k = 0; k < 3; ++k)
	    this->A ().insert (row + k, col + k) = 1.0;

	  if (isRelative)
	    {
	      const cnoid::Vector3& markerPosition =
		motion->frame(frameId)[activeVertexIndex];
	      this->b ().segment<3> (row) = markerPosition + pos;
	    }
	  else
	    this->b ().segment<3> (row) = pos;
	}

      this->A ().finalize ();


      // Print A in file (dense representation).
      {
	std::ofstream file
	  ((boost::format ("/tmp/position-%1%-A.dat") % localMarkerIndex).str ().c_str ());
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
	std::ofstream file
	  ((boost::format ("/tmp/position-%1%-b.dat") % localMarkerIndex).str ().c_str ());
	for (Eigen::MatrixXd::Index elt = 0; elt < b ().size (); ++elt)
	  file << b ()[elt] << "\n";
      }

      std::cout << "POSITION:" << pos << std::endl;
    }

    Position::~Position () throw ()
    {}
  } // end of namespace retargeting.
} // end of namespace roboptim.
