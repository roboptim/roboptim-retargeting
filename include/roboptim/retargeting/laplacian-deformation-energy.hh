#ifndef ROBOPTIM_RETARGETING_DEFORMATION_ENERGY_HH
# define ROBOPTIM_RETARGETING_DEFORMATION_ENERGY_HH
# include <vector>
# include <roboptim/core/linear-function.hh>

# include <roboptim/retargeting/laplacian-coordinate.hh>
# include <roboptim/retargeting/animated-interaction-mesh.hh>

namespace roboptim
{
  namespace retargeting
  {
    class LaplacianDeformationEnergy;

    typedef boost::shared_ptr<LaplacianDeformationEnergy>
    LaplacianDeformationEnergyShPtr_t;

    /// \brief Compute that Laplacian Deformation Energy.
    ///
    /// \f[
    /// E_L(V_{i}') = \sum_{j} \| \delta_{j} - L(p_{j}'^{i}) \|^{2}
    /// \f]
    class LaplacianDeformationEnergy : public roboptim::LinearFunction
    {
    public:
      explicit LaplacianDeformationEnergy
      (AnimatedInteractionMeshShPtr_t mesh) throw ();

      virtual ~LaplacianDeformationEnergy () throw ();
      void impl_compute (result_t& result, const argument_t& x)
	const throw ();
      void impl_gradient (gradient_t& gradient,
			  const argument_t& argument,
			  size_type functionId = 0)
	const throw ();
    private:
      /// \brief Original mesh.
      AnimatedInteractionMeshShPtr_t animatedMesh_;
      /// \brief Current state of the mesh (during optimization process).
      AnimatedInteractionMeshShPtr_t animatedMeshLocal_;
      /// \brief Original laplacian coordinates.
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >
      laplacianCoordinates_;
      std::vector<LaplacianCoordinateShPtr_t> laplacianCoordinatesLocal_;

      mutable Eigen::VectorXd buffer_;
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_DEFORMATION_ENERGY_HH
