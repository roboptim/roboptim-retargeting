#ifndef ROBOPTIM_RETARGETING_ACCELERATION_HH
# define ROBOPTIM_RETARGETING_ACCELERATION_HH


namespace roboptim
{
  namespace retargeting
  {
    /// \brief Compute acceleration from position.
    ///
    /// Use second derivative as acceleration.
    ///
    /// VectorInterpolation should be used here for implementation.
    template <typename T>
    boost::shared_ptr<GenericDifferentiableFunction<T> >
    acceleration ()
    {
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.


#endif //! ROBOPTIM_RETARGETING_ACCELERATION_HH
