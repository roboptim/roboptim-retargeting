`bin` directory
===============

This file contains source code for three programs. Each of them
processing motion data in a particular way. The source code in this
directory is only managing the optimization process at a very
high-level: building the problem, solving it, storing the result.

The optimization problems building is implemented in
`include/roboptim/retargeting/problem` and underlying mathematical
formula are implemented in `roboptim/retargeting/function`.
