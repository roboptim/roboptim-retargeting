#ifndef ROBOPTIM_RETARGETING_FUNCTION_BONE_LENGTH_ERROR_HH
# define ROBOPTIM_RETARGETING_FUNCTION_BONE_LENGTH_ERROR_HH
# include <algorithm>
# include <cassert>

# include <roboptim/core/numeric-quadratic-function.hh>
# include <roboptim/core/finite-difference-gradient.hh>

# include <cnoid/Body>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Compute one bone length error for one frame from
    ///        markers positions.
    ///
    /// Input: marker positions (number of markers * 3)
    /// Output: error (real number, size 1)
    ///
    /// This function is quadratic. It computes:
    ///
    /// \f$ B(\mathbf{x}) = \frac{1}{2} |x_{\text{ref}} - x|^2 \f$
    ///
    /// The desired bone length is the distance between the current
    /// body and its parent.
    ///
    /// \tparam T Function traits type
    template <typename T>
    class BoneLengthError : public GenericNumericQuadraticFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericNumericQuadraticFunction<T>);

      explicit BoneLengthError
      (const std::string& markerStart,
       const std::string& markerEnd,
       const std::string& linkName,
       const libmocap::MarkerTrajectory& markerTrajectory,
       cnoid::BodyPtr robot)
	: GenericNumericQuadraticFunction<T>
	  (matrix_t (3 * markerTrajectory.numMarkers (),
		     3 * markerTrajectory.numMarkers ()),
	   vector_t::Zero (3 * markerTrajectory.numMarkers ()),
	   vector_t::Zero (1))
      {
	if (!robot)
	  throw std::runtime_error ("null body pointer");

	// Fill matrix A.
	typename vector_t::Index markerStartId = 0;
	typename vector_t::Index markerEndId = 0;

	std::vector<std::string>::const_iterator it;

	it = std::find (markerTrajectory.markers ().begin (),
			markerTrajectory.markers ().end (),
			markerStart);
	if (it == markerTrajectory.markers ().end ())
	  throw std::runtime_error
	    ((boost::format ("start marker %s does not exist")
	      % markerStart).str ());

	it = std::find (markerTrajectory.markers ().begin (),
			markerTrajectory.markers ().end (),
			markerEnd);
	if (it == markerTrajectory.markers ().end ())
	  throw std::runtime_error
	    ((boost::format ("end marker %s does not exist")
	      % markerEnd).str ());

	assert (3 * markerStartId + 2 < this->A ().rows ()
		&& 3 * markerEndId + 2 < this->A ().rows ());

	this->A ().setZero ();

	// Put one in the diagonal elements matching the markers.
	this->A ().diagonal ().template segment<3> (3 * markerStartId).setIdentity ();
	this->A ().diagonal ().template segment<3> (3 * markerEndId).setIdentity ();

	// Put minus one for cross multiplication.
	this->A ().template block<3, 3>
	  (3 * markerStartId, 3 * markerStartId).diagonal ().fill (-1.);
	this->A ().template block<3, 3>
	  (3 * markerEndId, 3 * markerEndId).diagonal ().fill (-1.);

	// Compute desired bone length (value C).
	cnoid::Link* link = robot->link (linkName);
	if (!link)
	  throw std::runtime_error
	    ((boost::format ("link %s not found") % linkName).str ());
	cnoid::Link* parent = link->parent ();
	if (!parent)
	  throw std::runtime_error
	    ((boost::format ("link %s has no parent") % linkName).str ());

	this->c ()[0] =
	  (link->position ().translation ()
	   - parent->position ().translation ()).squaredNorm ();
      }

      virtual ~BoneLengthError ()
      {}
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_BONE_LENGTH_HH
