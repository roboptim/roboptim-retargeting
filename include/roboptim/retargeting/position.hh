#ifndef ROBOPTIM_RETARGETING_POSITION_HH
# define ROBOPTIM_RETARGETING_POSITION_HH
# include <boost/shared_ptr.hpp>
# include <roboptim/core/numeric-linear-function.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

# include <roboptim/retargeting/bone.hh>

# include <roboptim/retargeting/animated-interaction-mesh.hh>

# include <cnoid/ext/MocapPlugin/MarkerIMesh.h>

namespace roboptim
{
  namespace retargeting
  {
    class Position;
    typedef boost::shared_ptr<Position> PositionShPtr_t;

    /// \brief Position constraint (not implemented)
    class Position :
      public roboptim::GenericNumericLinearFunction<EigenMatrixSparse>
    {
    public:
      explicit Position
      (cnoid::MarkerIMeshPtr mesh,
       int motionIndex,
       int localMarkerIndex,
       const cnoid::Vector3& pos,
       bool isRelative) throw ();
      virtual ~Position () throw ();
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_POSITION_HH
