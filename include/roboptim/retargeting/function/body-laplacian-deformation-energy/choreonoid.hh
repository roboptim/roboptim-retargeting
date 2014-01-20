#ifndef ROBOPTIM_RETARGETING_FUNCTION_BODY_LAPLACIAN_DEFORMATION_ENERGY_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FUNCTION_BODY_LAPLACIAN_DEFORMATION_ENERGY_CHOREONOID_HH
# include <stdexcept>

# include <boost/make_shared.hpp>

# include <roboptim/core/differentiable-function.hh>
# include <roboptim/core/finite-difference-gradient.hh>
# include <roboptim/core/util.hh>

# include <roboptim/retargeting/function/joint-to-marker/choreonoid.hh>

# include <cnoid/BodyIMesh>

namespace roboptim
{
  namespace retargeting
  {
    template <typename T>
    std::ostream&
    printMatrix (std::ostream& o, const Eigen::MatrixBase<T>& matrix)
    {
      Eigen::IOFormat ioformat (Eigen::StreamPrecision,
				Eigen::DontAlignCols,
				",", ", ", "(", ")", "(", ")");
      ioformat.rowSpacer = "";
      o << "[";

      // Matrix
      if (matrix.cols () == 1 || matrix.cols () == 1)
	{
	  // Vector
	  ioformat = Eigen::IOFormat (Eigen::StreamPrecision,
				      Eigen::DontAlignCols,
                                    ",", ",", "", "", "(", ")");
	  ioformat.rowSpacer = "";
	  o << matrix.size ();
	}
      else
	o << matrix.rows () << "," << matrix.cols ();

      o << "]" << matrix.format (ioformat);
      return o;
    }

    template <typename T>
    std::ostream& operator<< (std::ostream& o, const std::set<T>& set)
    {
      typedef typename std::set<T>::const_iterator citer_t;

      if (set.empty ())
	return o << "Empty set";

      citer_t it = set.begin ();
      o << *it;
      ++it;

      for (; it != set.end (); ++it)
	o << ", " << *it;
      return o;
    }

    std::ostream& operator<< (std::ostream& o,
			      const cnoid::Link& link)
    {
      o << "Link " << link.name () << incindent << iendl;

      o << "Id: " << link.jointId () << iendl;
      o << "Type: " << link.jointType () << iendl;

      o << decindent;
      return o;
    }

    std::ostream& operator<< (std::ostream& o,
			      const cnoid::BodyMotion& bodyMotion)
    {
      o << "BodyMotion" << incindent << iendl;

      o << "Number of joints: " << bodyMotion.numJoints () << iendl;
      o << "Number of links: " << bodyMotion.numLinks () << iendl;
      o << "Frame rate: " << bodyMotion.frameRate () << iendl;
      o << "Number of frames: " << bodyMotion.numFrames () << iendl;

      o << decindent;
      return o;
    }

    std::ostream& operator<< (std::ostream& o,
			      const cnoid::BodyIMesh::Marker& marker)
    {
      o << "Link:" << incindent << iendl;
      if (marker.link)
	o << *marker.link;
      else
	o << "empty";
      o << decindent << iendl;

      o << "Local Position:" << incindent << iendl;
      if (marker.localPos)
	o << *marker.localPos;
      else
	o << "empty";
      o << decindent << iendl;

      o << "Neighbors to exclude:" << incindent << iendl
	<< marker.neighborsToExclude << decindent << iendl;

      return o;
    }

    std::ostream& operator<< (std::ostream& o,
			      const cnoid::BodyIMesh::BodyInfo& bodyInfo)
    {
      o << "Body:" << incindent << iendl;
      if (bodyInfo.body)
	o << bodyInfo.body;
      else
	o << "empty";
      o << decindent << iendl;

      o << "Motion:" << incindent << iendl;
      if (bodyInfo.motion)
	o << *(bodyInfo.motion);
      else
	o << "empty";
      o << decindent << iendl;

      o << "Number of markers: " << bodyInfo.markers.size () << iendl;

      for (int markerId = 0;
	   markerId < bodyInfo.markers.size (); ++markerId)
	o << "Marker id " << markerId << incindent << iendl
	  << (*bodyInfo.markers[markerId]) << decindent << iendl;

      o << "Marker Index Offset: " << bodyInfo.markerIndexOffset << iendl;
      return o;
    }


    std::ostream& operator<< (std::ostream& o, const cnoid::BodyIMesh& mesh)
    {
      //FIXME: workaround around bad API
      cnoid::BodyIMesh& meshNonConst = const_cast<cnoid::BodyIMesh&> (mesh);

      o << "Choreonoid Body Interaction Mesh" << iendl;

      o << "Number of bodies" << incindent << iendl
	<< mesh.numBodies () << decindent << iendl;

      o << "Number of vertices" << incindent << iendl
	<< mesh.numVertices () << decindent << iendl;

      o << "Number of markers" << incindent << iendl
	<< mesh.numMarkers () << decindent << iendl;

      o << "List of Bodies" << incindent << iendl;
      for (int bodyId = 0; bodyId < mesh.numBodies (); ++bodyId)
	{
	  const cnoid::BodyIMesh::BodyInfo& bodyInfo =
	    meshNonConst.bodyInfo (bodyId);
	  o << "Body number " << bodyId << incindent << iendl
	    << bodyInfo << decindent << iendl;
	}
      o << decindent;

      return o;
    }

    /// \brief Laplacian Deformation Energy computed using Choreonoid
    ///        Motion Capture plug-in.
    ///
    /// Input:
    ///  x = [q]
    ///
    /// Output:
    ///  result = [cost]
    ///
    /// \tparam T Function traits type
    template <typename T>
    class BodyLaplacianDeformationEnergyChoreonoid
      : public GenericDifferentiableFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericDifferentiableFunction<T>);

      explicit BodyLaplacianDeformationEnergyChoreonoid
      (cnoid::BodyIMeshPtr mesh,
       const std::size_t nDofs,
       const std::size_t nFrames,
       const vector_t& initialJointsTrajectory,
       boost::shared_ptr<GenericDifferentiableFunction<T> > jointToMarker,
       boost::shared_ptr<JointToMarkerPositionChoreonoid<T> >
       jointToMarkerOrigin)
	throw (std::runtime_error)
	: GenericDifferentiableFunction<T>
	  (nFrames * (6 + mesh->bodyInfo (0).body->numJoints ()), 1,
	   "BodyLaplacianDeformationEnergyChoreonoid"),
	  mesh_ (mesh),
	  nDofs_ (nDofs),
	  nFrames_ (nFrames),
	  jointToMarker_ (jointToMarker),
	  jointToMarkerOrigin_ (jointToMarkerOrigin),
	  markerPositions_ (mesh_->numMarkers () * 3),

	  originalLaplacianCoordinates_
	  (mesh_->getNumFrames () * mesh_->numMarkers () * 3),
	  currentLaplacianCoordinates_
	  (mesh_->getNumFrames () * mesh_->numMarkers () * 3)
      {
	if (mesh_->getNumFrames () != nFrames)
	  throw std::runtime_error ("invalid number of frames");
	if (initialJointsTrajectory.size () != this->inputSize ())
	  {
	    boost::format fmt
	      ("invalid size for initial joint trajectory"
	       " (expected is %d but size is %d");
	    fmt % (nFrames * nDofs) % initialJointsTrajectory.size ();
	    throw std::runtime_error (fmt.str ());
	  }

	// Set vectors to zero.
	markerPositions_.setZero ();
	originalLaplacianCoordinates_.setZero ();
	currentLaplacianCoordinates_.setZero ();

	// Compute initial Laplacian coordinates.
	computeLaplacianCoordinates
	  (originalLaplacianCoordinates_, initialJointsTrajectory);
	mesh_->update ();
	computeLaplacianCoordinates
	  (originalLaplacianCoordinates_, initialJointsTrajectory);
      }

      virtual ~BodyLaplacianDeformationEnergyChoreonoid () throw ()
      {}

      void computeLaplacianCoordinates (vector_t& result,
					const argument_t& x) const throw ()
      {
	result.setZero ();
	for (int frameId = 0; frameId < nFrames_; ++frameId)
	  {
	    const cnoid::BodyIMesh::Frame& neighborLists =
	      mesh_->frame (frameId);

	    // Compute the current marker position
	    markerPositions_.setZero ();
	    jointToMarkerOrigin_->frameId () = frameId;
	    jointToMarkerOrigin_->shouldUpdate ();
	    (*jointToMarker_)
	      (markerPositions_,
	       x.segment (frameId * nDofs_, nDofs_));

	    // Compute Laplacian Coordinates of each marker.
	    result.segment
	      (frameId * mesh_->numMarkers () * 3,
	       mesh_->numMarkers () * 3) = markerPositions_;

	    for (int markerId = 0; markerId < mesh_->numMarkers (); ++markerId)
	      {
	    	const cnoid::BodyIMesh::NeighborList&
	    	  neighbors = neighborLists[markerId];
	    	for (int l = 0; l < neighbors.size (); ++l)
	    	  {
	    	    int neighborId = neighbors[l];
	    	    double weight =
	    	      (markerPositions_.segment (markerId * 3, 3) -
	    	       markerPositions_.segment (neighborId * 3, 3)).norm ();
	    	    if (std::abs (weight) > 1e-8)
	    	      weight = 1. / weight;
		    weight = 1.; //FIXME: WEIGHTS ARE DISABLED FOR NOW !!!!!!!

	    	    result.segment
	    	      (frameId * mesh_->numMarkers () * 3 + markerId * 3, 3) -=
	    	      weight *
	    	      markerPositions_.segment (neighborId * 3, 3);
	    	  }
	      }
	  }
      }

    protected:
      void
      impl_compute
      (result_t& result, const argument_t& x)
	const throw ()
      {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	currentLaplacianCoordinates_.setZero ();

	// Compute Laplacian Coordinates for current x
	computeLaplacianCoordinates (currentLaplacianCoordinates_, x);

	// Compute Laplacian Deformation Energy
	result[0] =
	  (originalLaplacianCoordinates_ -
	   currentLaplacianCoordinates_).norm ();
      }

      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type i)
	const throw ()
      {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	roboptim::GenericFiniteDifferenceGradient<
	  T,
	  finiteDifferenceGradientPolicies::Simple<T> >
	  fdg (*this);
	fdg.gradient (gradient, x, i);
      }

      virtual std::ostream& print (std::ostream& o) const throw ()
      {
	o << this->getName () << iendl;

	o << "Interaction Mesh" << incindent << iendl;
	if (mesh_)
	  o << (*mesh_) << decindent << iendl;
	else
	  o << "empty";

	o << "Number of DOFs" << incindent << iendl
	  << nDofs_ << decindent << iendl;

	o << "Number of frames" << incindent << iendl
	  << nFrames_ << decindent << iendl;

	o << "Joint to marker" << incindent << iendl;
	if (jointToMarkerOrigin_)
	  o << (*jointToMarkerOrigin_);
	else
	  o << "empty";
	o << decindent << iendl;

	o << "Original Laplacian Coordinates" << incindent << iendl;
	printMatrix (o, originalLaplacianCoordinates_);
	o<< decindent << iendl;

	o << "Last Computed Laplacian Coordinates" << incindent << iendl;
	printMatrix (o, currentLaplacianCoordinates_);
	o << decindent << iendl;

	o << "Neighbors" << incindent << iendl;
	for (int frameId = 0; frameId < nFrames_; ++frameId)
	  {
	    o << "frame " << frameId << incindent << iendl;
	    const cnoid::BodyIMesh::Frame& neighborLists =
	      mesh_->frame (frameId);
	    for (int markerId = 0; markerId < mesh_->numMarkers (); ++markerId)
	      {
		o << "marker " << markerId << incindent << iendl;
		const cnoid::BodyIMesh::NeighborList&
		  neighbors = neighborLists[markerId];
		for (int l = 0; l < neighbors.size (); ++l)
		  {
		    if (l != 0)
		      o << ", ";
		    o << neighbors[l];
		  }
		o << decindent << iendl;
	      }
	    o << decindent << iendl;
	  }
	o << decindent << iendl;

	return o;
      }

    private:
      cnoid::BodyIMeshPtr mesh_;
      std::size_t nDofs_;
      std::size_t nFrames_;

      boost::shared_ptr<GenericDifferentiableFunction<T> > jointToMarker_;
      boost::shared_ptr<JointToMarkerPositionChoreonoid<T> >
      jointToMarkerOrigin_;
      mutable result_t markerPositions_;

      result_t originalLaplacianCoordinates_;
      mutable result_t currentLaplacianCoordinates_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_BODY_LAPLACIAN_DEFORMATION_ENERGY_CHOREONOID_HH
