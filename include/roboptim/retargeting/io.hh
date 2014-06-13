// Copyright (C) 2014 by Thomas Moulard, AIST, CNRS.
//
// This file is part of the roboptim.
//
// roboptim is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// roboptim is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with roboptim.  If not, see <http://www.gnu.org/licenses/>.

#ifndef ROBOPTIM_RETARGETING_INTERACTION_IO_HH
# define ROBOPTIM_RETARGETING_INTERACTION_IO_HH
# include <iostream>
# include <set>

# include <roboptim/core/function.hh> //FIXME: for Eigen

namespace roboptim
{
  namespace retargeting
  {
    template <typename T>
    std::ostream&
    printMatrix (std::ostream& o, const Eigen::MatrixBase<T>& matrix)
    {
      Eigen::IOFormat ioformat (Eigen::StreamPrecision,
				Eigen::DontAlignCols,
				",", ", ", "(", ")", "(", ")");
      ioformat.rowSpacer = "";
      o << "[";

      // Matrix
      if (matrix.cols () == 1 || matrix.cols () == 1)
	{
	  // Vector
	  ioformat = Eigen::IOFormat (Eigen::StreamPrecision,
				      Eigen::DontAlignCols,
                                    ",", ",", "", "", "(", ")");
	  ioformat.rowSpacer = "";
	  o << matrix.size ();
	}
      else
	o << matrix.rows () << "," << matrix.cols ();

      o << "]" << matrix.format (ioformat);
      return o;
    }

    template <typename T>
    std::ostream& operator<< (std::ostream& o, const std::set<T>& set)
    {
      typedef typename std::set<T>::const_iterator citer_t;

      if (set.empty ())
	return o << "Empty set";

      citer_t it = set.begin ();
      o << *it;
      ++it;

      for (; it != set.end (); ++it)
	o << ", " << *it;
      return o;
    }

  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_INTERACTION_IO_HH
