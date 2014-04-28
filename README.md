roboptim-retargeting
====================

Overview
--------

RobOptim toolbox aiming at processing Motion Capture Data and in
particular retarget motion capture data in order to allow a particular
robot to replace the motion.


This toolbox relies on the following software:

* Choreonoid (rigid body computation)
* libmocap (motion capture data loading)

Optionally, if metapod is available, some constraints have also been
implemented using this library.


It provides three command-line tools which are in fact the three steps
of the conversion process:

- roboptim-retargeting-marker-optimization
- roboptim-retargeting-marker-to-joint-converter
- roboptim-retargeting-joint-optimization


The marker optimization optimizes the markers positions in the
Euclidian space. If the motion capture matches a human skeleton, this
step will adapt the markers positions at each frame to make it match
the robot morphology.


Marker to Joint converter converts marker positions to joints
trajectories of a particular robot.


The joint optimization will, at least, reshape the joint trajectories
to satisfy robotics constraints such as balance (ZMP), actuators
limits (position, velocity, torque).


RobOptim Functions
------------------

All functions are placed into the
`include/roboptim/retargeting/function` folder.


### Acceleration (joint-based optimization and marker-based optimization)

    #include <roboptim/retargeting/function/acceleration.hh

Use StateFunction to compute each joint/marker acceleration over three
frames. This function can be used to penalize large motion as a cost
function.


### Marker Laplacian Deformation Energy (marker-based optimization)

    #include <roboptim/retargeting/function/marker-laplacian-deformation-energy/choreonoid.hh>

Compute the Marker Laplacian Deformation Energy for markers.


### Body Laplacian Deformation Energy (joint-based optimization)

    #include <roboptim/retargeting/function/body-laplacian-deformation-energy/choreonoid.hh>

Compute the Marker Laplacian Deformation Energy for bodies.


### Bone Length (marker-based optimization)

    #include <roboptim/retargeting/function/bone-length/choreonoid.hh>

Compute bone length for each segment and compare it to the robot segments.


### Joints limits (joint-based optimization)

    #include <roboptim/retargeting/function/joint-limits/choreonoid.hh>

Compute joints positions and/or velocities and make sure they are in
the actuators limits.


### Joints torque limits (joint-based optimization)

    #include <roboptim/retargeting/function/torque/choreonoid.hh>

Compute joints torques and make sure they are in the actuators limits.


### Body position (joint-based optimization)

    #include <roboptim/retargeting/function/forward-geometry/choreonoid.hh>

Force a particular body to stay at the correct location. Typically, it
is used to force the feet on the floor.


### ZMP (joint-based optimization)

    #include <roboptim/retargeting/function/zmp/choreonoid.hh>

Compute the Zero-Momentum Point attached with the robot. If it stays
in the support-polygon then the motion will remain balanced.


RobOptim Problems
-----------------

### Marker Optimization

Cost: Laplacian Deformation Energy (marker-based) and optionally acceleration.

Constraints:
- Bone Length (linear function)

### Joint Optimization

Cost: Laplacian Deformation Energy (joint-based) and optionally acceleration.

Constraints:
- Joint Limits (position, velocity)
- Body Position (feet on the ground)
- Torque
- ZMP
