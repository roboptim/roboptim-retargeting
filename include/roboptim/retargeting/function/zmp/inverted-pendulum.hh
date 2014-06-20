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

#ifndef ROBOPTIM_RETARGETING_FUNCTION_ZMP_INVERTED_PENDULUM_HH
# define ROBOPTIM_RETARGETING_FUNCTION_ZMP_INVERTED_PENDULUM_HH
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace retargeting
  {
    /// \brief ZMP position computed using the Inverted Pendulum
    ///        model.
    ///
    /// \tparam T Function traits type
    template <typename T>
    class ZMPInvertedPendulum : public ZMP<T>
    {
    public:
      ROBOPTIM_DIFFERENTIABLE_FUNCTION_FWD_TYPEDEFS_ (ZMP<T>);

      explicit ZMPInvertedPendulum (size_type nDofs)
	: ZMP<T> (nDofs, "choreonoid")
      {}

      virtual ~ZMPInvertedPendulum ()
      {}

    protected:
      void
      impl_compute
      (result_t& result, const argument_t& x)
	const
      {
	//FIXME: implement this
      }

      void
      impl_gradient (gradient_t& gradient,
		     const argument_t& x,
		     size_type i)
	const
      {
	//FIXME: implement this
      }
    };
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_FUNCTION_ZMP_INVERTED_PENDULUM_HH
