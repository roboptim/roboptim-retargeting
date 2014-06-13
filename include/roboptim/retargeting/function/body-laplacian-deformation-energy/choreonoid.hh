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

#ifndef ROBOPTIM_RETARGETING_FUNCTION_BODY_LAPLACIAN_DEFORMATION_ENERGY_CHOREONOID_HH
# define ROBOPTIM_RETARGETING_FUNCTION_BODY_LAPLACIAN_DEFORMATION_ENERGY_CHOREONOID_HH
# include <stdexcept>

# include <boost/make_shared.hpp>

# include <roboptim/core/differentiable-function.hh>
# include <roboptim/core/filter/chain.hh>
# include <roboptim/core/finite-difference-gradient.hh>
# include <roboptim/core/numeric-quadratic-function.hh>
# include <roboptim/core/util.hh>

# include <roboptim/retargeting/function/rss.hh>
# include <roboptim/retargeting/function/joint-to-marker/choreonoid.hh>
# include <roboptim/retargeting/function/laplacian-coordinate/choreonoid.hh>
# include <roboptim/retargeting/morphing.hh>
# include <roboptim/retargeting/utility.hh>
# include <roboptim/retargeting/io.hh>

namespace roboptim
{
  namespace retargeting
  {
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
    /// \f$ f(x) = \sum f_i(x_i) \f$
    ///
    /// \f$ f_i(x_i) = lde(laplacianCoordinate(jointToMarker(x))) \f$
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

      typedef boost::shared_ptr<RSS<T> >
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
      (MarkerMappingShPtr markerMapping,
       InteractionMeshShPtr mesh,
       TrajectoryShPtr trajectory,
       JointToMarkerShPtr_t jointToMarker)
	: GenericDifferentiableFunction<T>
	  (safeGet(trajectory).parameters ().size (), 1,
	   "BodyLaplacianDeformationEnergyChoreonoid"),

	  trajectory_ (safeGet (trajectory).clone ()),

	  mesh_ (mesh),

	  // Heuristic choice to decide how many discretization points
	  // should be considered when summing.
	  //
	  // Here we choose one per frame (in the discrete case) or
	  // per control point (parametrized curve).
	  nDiscretizationPoints_
	  (numberOfDiscretizationPoints (trajectory)),

	  jointToMarker_ (jointToMarker),
	  laplacianCoordinate_ (nDiscretizationPoints_),

	  lde_ (nDiscretizationPoints_),
	  chainLc_ (nDiscretizationPoints_),
	  chain_ (nDiscretizationPoints_),

	  markerPositions_
	  (safeGet (markerMapping).numMarkersEigen () * 3)
      {
	// Fill array and create necessary functions for each
	// discretization point.
	for (std::size_t p = 0; p < nDiscretizationPoints_; ++p)
	  {
	    StableTimePoint t = p / nDiscretizationPoints_ * tMax;

	    // Compute marker position for the original configuration.
	    (*jointToMarker)
	      (markerPositions_, safeGet (trajectory) (t));

	    // Create Laplacian Coordinate object, original markers positions
	    // are required to compute the weights.
	    laplacianCoordinate_[p] = boost::make_shared<
	      LaplacianCoordinateChoreonoid<T> >
	      (markerMapping, mesh, p, markerPositions_);

	    // Create the quadratic function computing ||A-X||^2
	    lde_[p] = boost::make_shared<RSS<T> >
	      ((*laplacianCoordinate_[p]) (markerPositions_));

	    // Chain the three functions together.
	    chainLc_[p] = roboptim::chain
	      <GenericDifferentiableFunction<T>,
	       GenericDifferentiableFunction<T>
	       >
	      (laplacianCoordinate_[p], jointToMarker_);
	    chain_[p] = roboptim::chain
	      <GenericDifferentiableFunction<T>,
	       GenericDifferentiableFunction<T>
	       >
	      (lde_[p], chainLc_[p]);
	  }
      }

      virtual ~BodyLaplacianDeformationEnergyChoreonoid ()
      {}

      /// \}

      /// \name Internal sub-objects getter for debugging purpose.
      /// \{

      const LaplacianCoordinatesShPtr_t&
      laplacianCoordinate () const
      {
	return laplacianCoordinate_;
      }

      const LaplacianDeformationEnergies_t&
      lde () const
      {
	return lde_;
      }

      const DifferentiableFunctionsShPtr_t&
      chainLc () const
      {
	return chainLc_;
      }

      const DifferentiableFunctionsShPtr_t&
      chain () const
      {
	return chain_;
      }

      /// \}


    protected:
      void
      impl_compute
      (result_t& result, const argument_t& x)
	const
      {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	ROBOPTIM_RETARGETING_ASSERT (chain_.size () == nDiscretizationPoints_);

	typename DifferentiableFunctionsShPtr_t::const_iterator it;

	trajectory_->setParameters (x);

	result.setZero ();

	std::size_t p = 0;
	for (it = chain_.begin (); it != chain_.end (); ++it, ++p)
	  {
	    ROBOPTIM_RETARGETING_ASSERT
	      ((*it)->inputSize () == safeGet (trajectory_).outputSize ());
	    ROBOPTIM_RETARGETING_ASSERT
	      ((*it)->outputSize () == result.size ());

	    StableTimePoint t = p / nDiscretizationPoints_ * tMax;
	    result += (*it)->operator () ((*trajectory_) (t));
	  }

	result *= .5;

	ROBOPTIM_RETARGETING_ASSERT
	  (static_cast<std::size_t> (p) == nDiscretizationPoints_);
      }

      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type)
	const
      {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
	Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

	ROBOPTIM_RETARGETING_ASSERT
	  (chain_.size () == nDiscretizationPoints_);

	typename DifferentiableFunctionsShPtr_t::const_iterator it;

	trajectory_->setParameters (x);

	gradient.setZero ();

	typename vector_t::Index p = 0;
	for (it = chain_.begin (); it != chain_.end (); ++it, ++p)
	  {
	    StableTimePoint t =
	      static_cast<std::size_t> (p) / nDiscretizationPoints_ * tMax;

	    gradient.segment
	      (p * trajectory_->outputSize (), trajectory_->outputSize ())
	      += (*it)->gradient ((*trajectory_) (t));
	  }

	gradient *= .5;

	ROBOPTIM_RETARGETING_ASSERT
	  (static_cast<std::size_t> (p) == nDiscretizationPoints_);
      }

      virtual std::ostream& print (std::ostream& o) const
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
	for (std::size_t p = 0; p < nDiscretizationPoints_; ++p)
	  {
	    o << "discretization point " << p << incindent << iendl;
	    const InteractionMesh::neighborsMap_t& neighbors =
	      safeGet(mesh_).neighbors (p);
	    for (InteractionMesh::neighborsMap_t::const_iterator
		   itMarker = neighbors.begin ();
		 itMarker != neighbors.end (); ++itMarker)
	      {
		o << "marker " << itMarker->first << incindent << iendl;
		for (InteractionMesh::neighbors_t::const_iterator
		       itNeighbors = itMarker->second.begin ();
		     itNeighbors != itMarker->second.end ();
		     ++itNeighbors)
		{
		  if (itNeighbors != itMarker->second.begin ())
		    o << ", ";
		  o << *itNeighbors;
		}
		o << decindent << iendl;
	      }
	    o << decindent << iendl;

	    // Avoid flooding the output, later a better solution should
	    // be found.
	    if (p >= 2)
	      {
		o << "[output too long, truncated]" << decindent << iendl;
		return o;
	      }
	  }
	o << decindent << iendl;

	return o;
      }

    private:
      /// \brief Copy of the initial trajectory.
      TrajectoryShPtr trajectory_;

      /// \brief Pointer to interaction mesh.
      InteractionMeshShPtr mesh_;

      /// \brief Number of discretization points (over time).
      std::size_t nDiscretizationPoints_;

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
