`problem` directory
===================

The problem directory contains the implementation of three
optimization problems (marker optimization, market to joint
conversion, joint optimization).

Each problem is divided into two parts:

* A problem builder: it loads data from files and instantiate the
  problem.
* A function builder: this is factory creating functions and
  constraints from their name.

Each problem has an "option" structure and a "data" structure. The
"option" structure contains the options to be used during the
optimization process. The "data" structure contains the data used by
the functions (trajectories, etc.). The data structure is built from
the options. Once the data structure is initialized, the functions can
be created and will rely on the data previously loaded.
