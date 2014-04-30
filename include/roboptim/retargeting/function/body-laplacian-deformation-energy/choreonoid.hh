#ifndef ROBOPTIM_RETARGETING_FUNCTION_BODY_LAPLACIAN_DEFORMATION_ENERGY_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FUNCTION_BODY_LAPLACIAN_DEFORMATION_ENERGY_CHOREONOID_HH
# include <stdexcept>

# include <boost/make_shared.hpp>

# include <roboptim/core/differentiable-function.hh>
# include <roboptim/core/filter/chain.hh>
# include <roboptim/core/finite-difference-gradient.hh>
# include <roboptim/core/numeric-quadratic-function.hh>
# include <roboptim/core/util.hh>

# include <roboptim/retargeting/function/laplacian-coordinate/choreonoid.hh>
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

      for (std::size_t markerId = 0;
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

    /// \brief Body Laplacian Deformation Energy (from laplacian
    ///        coordinates, for one frame)
    ///
    /// This function takes as input the Laplacian Coordinates of all
    /// markers concatenated for one frame.
    ///
    /// It returns the squared norm of the difference between this
    /// original value and the current input (i.e. the current
    /// Laplacian Coordinates) and is expressed as a quadratic
    /// function.
    ///
    /// f(x) = || S - X ||^2 = (S - X)^T . (S - X)
    ///      = S^2 - 2 . S . X + X^2
    ///
    /// A = I
    /// B = -2 * S
    /// C = S^T * S
    ///
    /// f(x) = X^T * I * X + B * X + C
    ///      = X^2 - 2 * S * X + S^2
    ///
    /// Input:
    /// x = [LC_x0, LC_y0, LC_z0, ..., LC_xN, LC_yN, LC_zN]
    ///
    /// Output:
    /// f(x) = [cost]
    ///
    /// \param originalLaplacianCoordinates Laplacian Coordinates of the
    ///        marker before any modification, this function will reach its
    ///        minimum value at this point.
    ///
    /// \tparam T Function traits type
    template <typename T>
    class BodyLaplacianDeformationEnergyChoreonoid2
      : public GenericNumericQuadraticFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericDifferentiableFunction<T>);

      explicit BodyLaplacianDeformationEnergyChoreonoid2
      (const vector_t& originalLaplacianCoordinates)
	throw (std::runtime_error) :
	GenericNumericQuadraticFunction<T>
	(matrix_t (originalLaplacianCoordinates.size (),
		   originalLaplacianCoordinates.size ()),
	 vector_t (originalLaplacianCoordinates.size ()))
      {
	this->A ().setIdentity ();

	this->b () = originalLaplacianCoordinates;
	this->b () *= -2.;

	this->c () =
	  originalLaplacianCoordinates.adjoint () * originalLaplacianCoordinates;
      }
    };

    /// \brief Laplacian Deformation Energy computed using Choreonoid
    ///        Motion Capture plug-in.
    ///
    /// The cost function is the sum for all the Laplacian Deformation Energies
    /// for all frames:
    ///
    /// Input:
    ///  x = [q]
    ///
    /// Output:
    ///  result = [cost]
    ///
    /// f(x) = \sum f_i(x_i)
    ///
    /// f_i(x_i) = lde(laplacianCoordinate(jointToMarker(x)))
    ///
    /// Implementation notes:
    /// ---------------------
    ///
    /// This function is implemented as the sum of three chained function.
    ///
    /// * JointToMarker is a non-linear function computing the marker
    ///   position from a specific configuration (for one frame).
    ///
    /// * LaplacianCoordinate computes the Laplacian Coordinates of each
    ///   marker (for one frame).
    ///
    /// * LaplacianDeformationEnergy (or lde) is computing the squared norm
    ///   of the difference between the current markers Laplacian Coordinates
    ///   and the reference Laplacian Coordinates. Most of the work is done
    ///   by the other sub-function so it is only defined as a quadratic
    ///   function computing .5 * ||A - X||^2 where A is set to the original
    ///   Laplacian Coordinates of the markers.
    ///
    /// Efficiency:
    /// -----------
    ///
    /// This function could be optimized by removing the use of the
    /// chain filter to avoid computing JointToMarker several
    /// times. Actually, the smarter way would be to change the code
    /// so that a frame-by-frame joint to marker function could be
    /// used.
    ///
    /// \tparam T Function traits type
    template <typename T>
    class BodyLaplacianDeformationEnergyChoreonoid
      : public GenericDifferentiableFunction<T>
    {
    public:
      /// \name Useful type aliases.
      /// \{

      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericDifferentiableFunction<T>);

      typedef boost::shared_ptr<JointToMarkerPositionChoreonoid<T> >
      JointToMarkerShPtr_t;

      typedef boost::shared_ptr<LaplacianCoordinateChoreonoid<T> >
      LaplacianCoordinateShPtr_t;
      typedef std::vector<LaplacianCoordinateShPtr_t>
      LaplacianCoordinatesShPtr_t;

      typedef boost::shared_ptr<BodyLaplacianDeformationEnergyChoreonoid2<T> >
      LaplacianDeformationEnergy_t;
      typedef std::vector<LaplacianDeformationEnergy_t>
      LaplacianDeformationEnergies_t;

      typedef boost::shared_ptr<GenericDifferentiableFunction<T> >
      DifferentiableFunctionShPtr_t;

      typedef std::vector<DifferentiableFunctionShPtr_t>
      DifferentiableFunctionsShPtr_t;

      /// \}


      /// \name Constructor, destructor.
      /// \{

      /// \brief Build the cost function.
      ///
      /// \param mesh Interaction Mesh
      /// \param initialJointsTrajectory initial articular trajectory
      ///        for the whole motion (all frames).
      ///
      /// \param jointToMarker shared pointer to the JointToMarker
      ///        function.
      explicit BodyLaplacianDeformationEnergyChoreonoid
      (cnoid::BodyIMeshPtr mesh,
       const vector_t& initialJointsTrajectory,
       JointToMarkerShPtr_t jointToMarker)
	throw (std::runtime_error)
	: GenericDifferentiableFunction<T>
	  (initialJointsTrajectory.size (), 1,
	   "BodyLaplacianDeformationEnergyChoreonoid"),
	  mesh_ (mesh),
	  nFrames_ (static_cast<size_type> (mesh->getNumFrames ())),
	  nDofs_ (initialJointsTrajectory.size () / nFrames_),
	  jointToMarker_ (jointToMarker),
	  laplacianCoordinate_ (mesh->getNumFrames ()),
	  lde_ (mesh->getNumFrames ()),
	  chainLc_ (mesh->getNumFrames ()),
	  chain_ (mesh->getNumFrames ()),
	  markerPositions_ (mesh->numMarkers () * 3)
      {
	// Fill array and create necessary functions for each frame.
	for (std::size_t frameId = 0; frameId < nFrames_; ++frameId)
	  {
	    // Compute marker position for the original configuration.
	    (*jointToMarker)
	      (markerPositions_,
	       initialJointsTrajectory.segment (frameId * nDofs_, nDofs_));

	    // Create Laplacian Coordinate object, original markers positions
	    // are required to compute the weights.
	    laplacianCoordinate_[frameId] = boost::make_shared<
	      LaplacianCoordinateChoreonoid<T> >
	      (mesh, frameId, markerPositions_);

	    // Create the quadratic function computing ||A-X||^2
	    lde_[frameId] = boost::make_shared<
	      BodyLaplacianDeformationEnergyChoreonoid2<T> >
	      ((*laplacianCoordinate_[frameId]) (markerPositions_));

	    // Chain the three functions together.
	    chainLc_[frameId] = roboptim::chain
	      <GenericDifferentiableFunction<T>,
	       GenericDifferentiableFunction<T>
	       >
	      (laplacianCoordinate_[frameId], jointToMarker_);
	    chain_[frameId] = roboptim::chain
	      <GenericDifferentiableFunction<T>,
	       GenericDifferentiableFunction<T>
	       >
	      (lde_[frameId], chainLc_[frameId]);
	  }
      }

      virtual ~BodyLaplacianDeformationEnergyChoreonoid () throw ()
      {}

      /// \}

      /// \name Internal sub-objects getter for debugging purpose.
      /// \{

      const LaplacianCoordinatesShPtr_t&
      laplacianCoordinate () const throw ()
      {
	return laplacianCoordinate_;
      }

      const LaplacianDeformationEnergies_t&
      lde () const throw ()
      {
	return lde_;
      }

      const DifferentiableFunctionsShPtr_t&
      chainLc () const throw ()
      {
	return chainLc_;
      }

      const DifferentiableFunctionsShPtr_t&
      chain () const throw ()
      {
	return chain_;
      }

      /// \}


    protected:
      void
      impl_compute
      (result_t& result, const argument_t& x)
	const throw ()
      {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	assert (chain_.size () == nFrames_);

	typename DifferentiableFunctionsShPtr_t::const_iterator it;

	result.setZero ();

	std::size_t frameId = 0;
	for (it = chain_.begin (); it != chain_.end (); ++it, ++frameId)
	  {
	    assert ((*it)->inputSize () == nDofs_);
	    assert ((*it)->outputSize () == result.size ());
	    result += (*it)->operator ()
	      (x.segment (frameId * nDofs_, nDofs_));
	  }

	result *= .5;

	assert (frameId == nFrames_);
      }

      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type)
	const throw ()
      {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	assert (chain_.size () == nFrames_);

	typename DifferentiableFunctionsShPtr_t::const_iterator it;

	gradient.setZero ();

	std::size_t frameId = 0;
	for (it = chain_.begin (); it != chain_.end (); ++it, ++frameId)
	  {
	    gradient.segment (frameId * nDofs_, nDofs_)
	      += (*it)->gradient
	      (x.segment (frameId * nDofs_, nDofs_));
	  }

	gradient *= .5;

	assert (frameId == nFrames_);
      }

      virtual std::ostream& print (std::ostream& o) const throw ()
      {
	o << this->getName () << iendl;

	o << "Interaction Mesh" << incindent << iendl;
	if (mesh_)
	  o << (*mesh_) << decindent << iendl;
	else
	  o << "empty";

	o << "Joint to marker" << incindent << iendl;
	if (jointToMarker_)
	  o << (*jointToMarker_);
	else
	  o << "empty";
	o << decindent << iendl;

	o << "Neighbors" << incindent << iendl;
	for (int frameId = 0; frameId < mesh_->getNumFrames (); ++frameId)
	  {
	    o << "frame " << frameId << incindent << iendl;
	    const cnoid::BodyIMesh::Frame& neighborLists =
	      mesh_->frame (frameId);
	    for (int markerId = 0; markerId < mesh_->numMarkers (); ++markerId)
	      {
		o << "marker " << markerId << incindent << iendl;
		const cnoid::BodyIMesh::NeighborList&
		  neighbors = neighborLists[markerId];
		for (std::size_t l = 0; l < neighbors.size (); ++l)
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
      /// \brief Pointer to interaction mesh.
      cnoid::BodyIMeshPtr mesh_;

      /// \brief Number of frames
      std::size_t nFrames_;
      /// \brief Number of DOFs
      size_type nDofs_;

      /// \brief Pointer to joint to marker function.
      JointToMarkerShPtr_t jointToMarker_;

      /// \brief Array of Laplacian Coordinates (one for each frame).
      ///
      /// Weights depend on the frame so we keep one per frame.
      LaplacianCoordinatesShPtr_t laplacianCoordinate_;

      /// \brief Array of Laplacian Deformation Energy (one for each
      ///        frame).
      ///
      /// This function computes || A - X ||^2 and hence is frame
      /// independent.
      ///
      /// A is frame dependent so we keep one per frame.
      LaplacianDeformationEnergies_t lde_;

      /// \brief Laplacian Coordinate from configuration.
      ///
      /// For each frame:
      /// f(x) = laplacianCoordinate(jointToMarker(x))
      ///
      /// roboptim-core "chain" filter is used here.
      DifferentiableFunctionsShPtr_t chainLc_;

      /// \brief Final computation expressed through a chain.
      ///
      /// For each frame:
      /// f(x) = lde_(chainLc(x))
      ///
      /// i.e.
      ///  f(x) = lde(laplacianCoordinate(jointToMarker(x)))
      ///
      ///
      /// roboptim-core "chain" filter is used here.
      DifferentiableFunctionsShPtr_t chain_;

      /// \brief Mutable buffer to store the marker position as
      ///        computed by jointToMarker for the current frame.
      mutable result_t markerPositions_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_BODY_LAPLACIAN_DEFORMATION_ENERGY_CHOREONOID_HH
