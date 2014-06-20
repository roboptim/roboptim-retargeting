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
