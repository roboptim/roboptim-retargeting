`src` directory
===============

There is not library built by this package due to the fact that all
the functions are templated. This open the possibility to switch from
dense to sparse problem quite easily. The only files here are headers
that should not be installed. For instance `directories.hh` contains
path definitions. The symbol may be used but this header should never
be included directly by external packages and hence is not
distributed.
