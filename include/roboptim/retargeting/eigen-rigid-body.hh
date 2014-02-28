#ifndef ROBOPTIM_RETARGETING_EIGEN_RIGID_BODY_HH
# define ROBOPTIM_RETARGETING_EIGEN_RIGID_BODY_HH
// FIXME: to include Eigen correctly.
# include <roboptim/core/function.hh>

/// \brief Set the rotational part of a transformation from
///        Euler angles parameters.
///
/// \b Formula
///
/// \f$\alpha\f$ is the yaw parameter (rotation around Z axis)
/// \f$\beta\f$ is the pitch parameter (rotation around Y axis)
/// \f$\gamma\f$ is the roll parameter (rotation around X axis)
///
/// \f$ \left( \begin{array}{ccc}
/// \cos(\alpha) \cos(\beta) &
/// \cos(\alpha) \sin(\beta) \sin(\gamma) - \sin(\alpha) \cos(\gamma) &
/// \cos(\alpha) \sin(\beta) \cos(\gamma) + \sin(\alpha) \sin(\gamma) \\
/// \sin(\alpha) \cos(\beta) &
/// \sin(\alpha) \sin(\beta) \sin(\gamma) + \cos(\alpha) \cos(\gamma) &
/// \sin(\alpha) \sin(\beta) \cos(\gamma) - \cos(\alpha) \sin(\gamma) \\
/// -\sin(\beta) &
/// \cos(\beta) \sin(\gamma) &
/// \cos(\beta) \cos(\gamma)
/// \end{array} \right) \f$
///
/// \b Sources
///
/// http://planning.cs.uiuc.edu/node102.html
///
/// This implementation is an adaptation of the one in sot-core.
///
/// \param[out] eulerAngles vector \f$(\gamma, \beta, \alpha)\f$
/// \param[in] transform store results
template <typename Derived, typename OtherDerived>
void
eulerToTransform (Eigen::MatrixBase<Derived> const& transform,
		  const Eigen::MatrixBase<OtherDerived>& eulerAngles);

/// \brief Set the rotational part of a transformation from
///        U\f$\theta\f$ parameters (three parameters version).
///
///
/// The three parameters encodes the rotation axis:
///
/// \f$\vec{v} = (\alpha, \beta, \gamma)\f$
///
/// The vector norm encodes the amount of rotation around the rotation
/// axis.
///
/// A notation exception is that if the norm is inferior to a
/// threshold the rotation matrix is set to identity.
///
/// \param[out] utheta vector \f$(\gamma, \beta, \alpha)\f$
/// \param[in] transform store results
template <typename Derived, typename OtherDerived>
void
uthetaToTransform (Eigen::MatrixBase<Derived> const& transform,
		   const Eigen::MatrixBase<OtherDerived>& utheta);


/// \brief Compute Euler angles parameters from a rotation matrix.
///
/// \b Notation
///
/// \f$\alpha\f$ is the yaw parameter (rotation around Z axis)
/// \f$\beta\f$ is the pitch parameter (rotation around Y axis)
/// \f$\gamma\f$ is the roll parameter (rotation around X axis)
///
/// \b Yaw
/// \f$ \alpha = \tan^{-1}(r_{10} / r_{00}) \f$
///
/// \b Pitch
/// \f$ \beta = \tan^{-1}(-r_{20} / \sqrt{r_{21}^2 + r_{22}^2}) \f$
///
/// \b Roll
/// \f$ \gamma = \tan^{-1}(r_{21} / r_{22}) \f$
///
/// \b Sources
///
/// http://planning.cs.uiuc.edu/node103.html
///
/// This implementation is an adaptation of the one in sot-core.
///
/// \param[out] eulerAngles vector \f$(\gamma, \beta, \alpha)\f$
/// \param[in] linear rotation matrix
template <typename Derived, typename OtherDerived>
void
transformToEuler (Eigen::MatrixBase<Derived> const& eulerAngles,
		  const Eigen::MatrixBase<OtherDerived>& linear);

/// \brief Compute U\f$\theta\f$ parameters from a rotation matrix.
///
/// \f$\vec{v} = (\alpha, \beta, \gamma)\f$
///
/// This relies on the Eigen::AngleAxis class.
///
/// \param[out] utheta vector \f$(\gamma, \beta, \alpha)\f$
/// \param[in] linear rotation matrix
template <typename Derived, typename OtherDerived>
void
transformToUTheta (Eigen::MatrixBase<Derived> const& utheta,
		   const Eigen::MatrixBase<OtherDerived>& linear);

/// \brief Build a skew-symmetric matrix from vector of size 3.
///
/// \b Formula
///
/// \f$ \left( \begin{array}{ccc}
/// 0 &
/// -\mathbf{v}_2 &
/// \mathbf{v}_1 \\
/// \mathbf{v}_2 &
/// 0 &
/// -\mathbf{v}_0 \\
/// -\mathbf{v}_1 &
/// \mathbf{v}_0 &
/// 0
/// \end{array} \right) \f$
///
/// \param[out] result matrix (size 3x3)
/// \param[in] vector \f$(\mathbf{v}_0, \mathbf{v}_1, \mathbf{v}_2)\f$
template <typename Derived, typename OtherDerived>
void hat (Eigen::MatrixBase<Derived> const& result,
	  const Eigen::MatrixBase<OtherDerived>& p);

/// \brief Build a skew-symmetric matrix from vector of size 3.
///
/// \see hat(eulerAngles, transform)
/// \param[out] result matrix
/// \param[in] vector \f$(\mathbf{v}_0, \mathbf{v}_1, \mathbf{v}_2)\f$
template <typename Derived>
Eigen::Matrix<typename Derived::RealScalar, 3, 3> hat
(const Eigen::MatrixBase<Derived>& p);

/// \brief Build a transformation adjoint.
///
/// It must be applied to a twist written as:
/// \f$ \mathbf{\xi} = (\mathbf{v}, \mathbf{\omega}) \f$
/// where \f$\mathbf{v}\f$ is the
///
/// \b Formula
///
/// \f$\mathbf{R}\f$ is the transformation rotation matrix (3x3).
/// \f$\mathbf{p}\f$ is the transformation translation vector (size 3).
///
/// \f$ \left( \begin{array}{cc}
/// \mathbf{R} & \hat{\mathbf{p}} \mathbf{R} \\
/// \mathbf{0} & \mathbf{R}
/// \end{array} \right) \f$
///
/// \param[out] adjoint adjoint matrix (size 6x6)
/// \param[in]  transform transformation
template <typename Derived, int _Dim, int _Mode, int _Options>
void
adjoint (Eigen::MatrixBase<Derived> const& adjoint,
	 const Eigen::Transform<typename Derived::RealScalar,
				_Dim, _Mode, _Options>& transform);

/// \brief Store a transformation as a vector.
///
/// The output vector contains three parameters for translation and
/// three for the rotation (Euler angles).
///
/// \f$v = (x, y, z, \gamma, \beta, \alpha)\f$
///
/// \see eulerToTransform transformToEuler
/// \param[out] result result vector
///                    \f$v = (x, y, z, \gamma, \beta, \alpha)\f$
/// \param[in]  transform input transformation
template <typename Derived, int _Dim, int _Mode, int _Options>
void
transformToVector (Eigen::MatrixBase<Derived> const& result,
		   const Eigen::Transform<typename Derived::RealScalar,
					  _Dim, _Mode, _Options>& transform);

/// \brief Store a transformation as a vector.
///
/// The output vector contains three parameters for translation and
/// three for the rotation (\f$U\theta\f$).
///
/// \f$v = (x, y, z, \gamma, \beta, \alpha)\f$
///
/// \see uthetaToTransform transformToUtheta
/// \param[out] result result vector
///                    \f$v = (x, y, z, \gamma, \beta, \alpha)\f$
/// \param[in]  transform input transformation
template <typename Derived, int _Dim, int _Mode, int _Options>
void
transformToUtheta (Eigen::MatrixBase<Derived> const& result,
		       const Eigen::Transform<typename Derived::RealScalar,
					      _Dim, _Mode, _Options>& transform);

/// \brief Convert a vector to a transform.
///
/// The input vector must three parameters for translation and
/// three for the rotation (Euler angles).
///
/// \f$v = (x, y, z, \gamma, \beta, \alpha)\f$
///
/// \see eulerToTransform transformToEuler
/// \param[out] transform output transformation
/// \param[in]  parameters vector
///             \f$v = (x, y, z, \gamma, \beta, \alpha)\f$
template <typename Derived, int _Dim, int _Mode, int _Options>
void
vectorToTransform (Eigen::Transform<typename Derived::RealScalar,
				    _Dim, _Mode, _Options>& transform,
		   const Eigen::MatrixBase<Derived>& result);

/// \brief Convert a vector to a transform.
///
/// The input vector must three parameters for translation and
/// three for the rotation (U\f$\theta\f$).
///
/// \f$v = (x, y, z, \gamma, \beta, \alpha)\f$
///
/// \see uthetaToTransform transformToUTheta
/// \param[out]  transform output transformation
/// \param[in]   parameters vector
///              \f$v = (x, y, z, \gamma, \beta, \alpha)\f$
template <typename Derived, int _Dim, int _Mode, int _Options>
void
uthetaPoseToTransform (Eigen::Transform<typename Derived::RealScalar,
					_Dim, _Mode, _Options>& transform,
		       const Eigen::MatrixBase<Derived>& result);

# include <roboptim/retargeting/eigen-rigid-body.hxx>
#endif //! ROBOPTIM_RETARGETING_EIGEN_RIGID_BODY_HH
