#ifndef ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_HH
# define ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_HH
# include <roboptim/core/linear-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Compute the offset between one joint and one or more
    /// attached markers.
    ///
    /// This function is linear as the joint/marker offset is constant.
    template <typename T>
    class JointToMarkerPosition : public GenericDifferentiableFunction<T>
    {
    public:
      explicit JointToMarkerPosition ()
	: GenericDifferentiableFunction ("JointToMarkerPosition", 1, 1*3)
      {
      }

      ~JointToMarkerPosition ()
      {
      }
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_JOINT_TO_MARKER_POSITION_HH
