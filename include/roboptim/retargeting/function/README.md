`function` directory
====================

This folder contains various RobOptim functions realizing particular
computations.

Folders match functions for which two or more implementation exist.

Usually, a common interface is also provided. For instance, for ZMP
computation:

* `zmp.hh` provide an abstract interface (all ZMP functions takes 3 *
  the size of the configuration as input and returns two values).
* `zmp/choreonoid.hh` is the implementation of a ZMP function using
  Choreonoid.

The goal here is to stay independent from the underlying library
realizing the computation and/or to switch easily from one strategy to
another when several approaches exist to compute a quantity.
