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

#ifndef ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_HH
# define ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_HH
# include <boost/format.hpp>
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Compute a particular body position for a given configuration.
    ///
    /// Input: [q0, q1, ..., qN]
    /// Output: [bodyM_x, bodyM_y, bodyM_z]
    template <typename T>
    class ForwardGeometry : public GenericDifferentiableFunction<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_
      (GenericDifferentiableFunction<T>);

      explicit ForwardGeometry (size_type nDofs,
				std::string title)

	: GenericDifferentiableFunction<T>
	  (nDofs, 6, (boost::format ("ForwardGeometry [%s]") % title).str ())
      {}

      virtual ~ForwardGeometry ()
      {}

      template <typename Derived>
      Eigen::VectorBlock<const Derived>
      translation (const Eigen::MatrixBase<Derived>& x) const
      {
	return x.segment (0, 3);
      }

      template <typename Derived>
      Eigen::VectorBlock<Derived>
      translation (Eigen::MatrixBase<Derived>& x) const
      {
	return x.segment (0, 3);
      }

      template <typename Derived>
      Eigen::VectorBlock<const Derived>
      rotation (const Eigen::MatrixBase<Derived>& x) const
      {
	return x.segment (3, 3);
      }

      template <typename Derived>
      Eigen::VectorBlock<Derived>
      rotation (Eigen::MatrixBase<Derived>& x) const
      {
	return x.segment (3, 3);
      }

      template <typename Derived>
      Eigen::VectorBlock<const Derived>
      q (const Eigen::MatrixBase<Derived>& x,
	 bool withBaseLink = true) const
      {
	std::size_t offset = 0;
	if (!withBaseLink)
	  offset += 6;
	return x.segment (offset, x.size () - offset);
      }

      template <typename Derived>
      Eigen::VectorBlock<Derived>
      q (Eigen::MatrixBase<Derived>& x,
	 bool withBaseLink = true) const
      {
	std::size_t offset = 0;
	if (!withBaseLink)
	  offset += 6;
	return x.segment (offset, x.size () - offset);
      }

    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FORWARD_GEOMETRY_HH
