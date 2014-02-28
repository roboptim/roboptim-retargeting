#ifndef ROBOPTIM_RETARGETING_EIGEN_RIGID_BODY_HXX
# define ROBOPTIM_RETARGETING_EIGEN_RIGID_BODY_HXX
# include <cmath>

template <typename Derived, typename OtherDerived>
void
eulerToTransform (Eigen::MatrixBase<Derived> const& linear,
		  const Eigen::MatrixBase<OtherDerived>& eulerAngles)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE (Derived, 3, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE (OtherDerived, 3);

  // cr means cos roll, sy sin yaw, etc.
  // sincos computes sinus and cosinus in parallel
  double cr, sr, cp, sp, cy, sy;
  sincos (eulerAngles[0], &sr, &cr);
  sincos (eulerAngles[1], &sp, &cp);
  sincos (eulerAngles[2], &sy, &cy);

  Eigen::MatrixBase<Derived>& linear_ =
    const_cast<Eigen::MatrixBase<Derived>&> (linear);

  linear_ (0, 0) = cy * cp;
  linear_ (0, 1) = cy * sp * sr - sy * cr;
  linear_ (0, 2) = cy * sp * cr + sy * sr;
  linear_ (1, 0) = sy * cp;
  linear_ (1, 1) = sy * sp * sr + cy * cr;
  linear_ (1, 2) = sy * sp * cr - cy * sr;
  linear_ (2, 0) = -sp;
  linear_ (2, 1) = cp * sr;
  linear_ (2, 2) = cp * cr;
}

template <typename Derived, typename OtherDerived>
void
uthetaToTransform (Eigen::MatrixBase<Derived> const& linear,
		   const Eigen::MatrixBase<OtherDerived>& utheta)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE (Derived, 3, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE (OtherDerived, 3);

  Eigen::MatrixBase<Derived>& linear_ =
    const_cast<Eigen::MatrixBase<Derived>&> (linear);

  typename Derived::RealScalar angle = utheta.norm ();

  if (angle < 1e-6)
    linear_.setIdentity ();
  else
    {
      Eigen::Matrix<typename Derived::RealScalar, 3, 1> axis = utheta / angle;
      Eigen::AngleAxis<typename Derived::RealScalar> angleAxis (angle, axis);
      angleAxis.toRotationMatix (linear_);
    }
}

template <typename Derived, typename OtherDerived>
void
transformToEuler (Eigen::MatrixBase<Derived> const& eulerAngles,
		  const Eigen::MatrixBase<OtherDerived>& linear)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE (Derived, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE (OtherDerived, 3, 3);

  Eigen::MatrixBase<Derived>& eulerAngles_ =
    const_cast<Eigen::MatrixBase<Derived>&> (eulerAngles);

  const double nx = linear (2, 2);
  const double ny = linear (2, 1);

  eulerAngles_[0] = std::atan2 (ny, nx);
  eulerAngles_[1] = std::atan2 (-linear (2, 0),
			       std::sqrt (ny * ny + nx * nx));
  eulerAngles_[2] = std::atan2 (linear (1, 0), linear (0, 0));
}

template <typename Derived, typename OtherDerived>
void
transformToUTheta (Eigen::MatrixBase<Derived> const& utheta,
		   const Eigen::MatrixBase<OtherDerived>& linear)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE (Derived, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE (OtherDerived, 3, 3);

  Eigen::MatrixBase<Derived>& utheta_ =
    const_cast<Eigen::MatrixBase<Derived>&> (utheta);

  Eigen::AngleAxis<typename Derived::RealScalar> angleAxis (linear);

  utheta_ = angleAxis.axis ();
  utheta_ *= angleAxis.angle ();
}


template <typename Derived, typename OtherDerived>
void
hat (Eigen::MatrixBase<Derived> const& result,
     const Eigen::MatrixBase<OtherDerived>& p)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE (Derived, 3, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE (OtherDerived, 3);

  Eigen::MatrixBase<Derived>& result_ =
    const_cast<Eigen::MatrixBase<Derived>&> (result);

  result_ (0, 0) = 0.;
  result_ (0, 1) = -p[2];
  result_ (0, 2) = p[1];

  result_ (1, 0) = p[2];
  result_ (1, 1) = 0.;
  result_ (1, 2) = -p[0];

  result_ (2, 0) = -p[1];
  result_ (2, 1) = p[0];
  result_ (2, 2) = 0.;
}

template <typename Derived>
Eigen::Matrix<typename Derived::RealScalar, 3, 3>
hat (const Eigen::MatrixBase<Derived>& p)
{
  Eigen::Matrix<typename Derived::RealScalar, 3, 3> result;
  hat (result, p);
  return result;
}

template <typename Derived, int _Dim, int _Mode, int _Options>
void
adjoint (Eigen::MatrixBase<Derived> const& adjoint,
	 const Eigen::Transform<typename Derived::RealScalar,
				_Dim, _Mode, _Options>& transform)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE (Derived, 6, 6);

  Eigen::MatrixBase<Derived>& adjoint_ =
    const_cast<Eigen::MatrixBase<Derived>&> (adjoint);

  adjoint_.template block<3, 3> (0, 0) = transform.linear ();

  hat (adjoint_.template block<3, 3> (0, 3), transform.translation ());
  adjoint_.template block<3, 3> (0, 3) *= transform.linear ();

  adjoint_.template block<3, 3> (3, 0).setZero ();
  adjoint_.template block<3, 3> (3, 3) = transform.linear ();
}

template <typename Derived, int _Dim, int _Mode, int _Options>
void
transformToVector (Eigen::MatrixBase<Derived> const& result,
		   const Eigen::Transform<typename Derived::RealScalar,
					  _Dim, _Mode, _Options>& transform)
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY (Derived);
  assert (result.size () == 6);

  Eigen::MatrixBase<Derived>& result_ =
    const_cast<Eigen::MatrixBase<Derived>&> (result);

  result_.template segment<3> (0) = transform.translation ();
  transformToEuler (result_.template segment<3> (3), transform.linear ());
}

template <typename Derived, int _Dim, int _Mode, int _Options>
void
transformToUtheta (Eigen::MatrixBase<Derived> const& result,
		       const Eigen::Transform<typename Derived::RealScalar,
					      _Dim, _Mode, _Options>& transform)
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY (Derived);
  assert (result.size () == 6);

  Eigen::MatrixBase<Derived>& result_ =
    const_cast<Eigen::MatrixBase<Derived>&> (result);

  result_.template segment<3> (0) = transform.translation ();
  transformToUTheta (result_.template segment<3> (3), transform.linear ());
}


template <typename Derived, int _Dim, int _Mode, int _Options>
void
vectorToTransform (Eigen::Transform<typename Derived::RealScalar,
				    _Dim, _Mode, _Options>& transform,
		   const Eigen::MatrixBase<Derived>& result)
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY (Derived);
  assert (result.size () == 6);

  transform.translation () = result.template segment<3> (0);
  eulerToTransform (transform.linear (), result.template segment<3> (3));
}


template <typename Derived, int _Dim, int _Mode, int _Options>
void
uthetaPoseToTransform (Eigen::Transform<typename Derived::RealScalar,
					_Dim, _Mode, _Options>& transform,
		       const Eigen::MatrixBase<Derived>& result)
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY (Derived);
  assert (result.size () == 6);

  transform.translation () = result.template segment<3> (0);
  uthetaToTransform (transform.linear (), result.template segment<3> (3));
}

#endif //! ROBOPTIM_RETARGETING_EIGEN_RIGID_BODY_HXX
