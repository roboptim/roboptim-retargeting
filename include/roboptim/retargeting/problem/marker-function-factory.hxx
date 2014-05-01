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

#ifndef ROBOPTIM_RETARGETING_MARKER_FUNCTION_FACTORY_HXX
# define  ROBOPTIM_RETARGETING_MARKER_FUNCTION_FACTORY_HXX
# include <stdexcept>

# include <roboptim/core/numeric-linear-function.hh>

//# include <roboptim/retargeting/function/bone-length.hh>
# include <roboptim/retargeting/function/marker-laplacian-deformation-energy/choreonoid.hh>

namespace roboptim
{
  namespace retargeting
  {
    namespace detail
    {
      template <typename T>
      boost::shared_ptr<T>
      null (const MarkerFunctionData& data)
      {
	if (!data.trajectory)
	  throw std::runtime_error
	    ("failed to create null function: no joint trajectory");

	typename T::matrix_t A
	  (1, data.trajectory->parameters ().size ());
	A.setZero ();
	typename T::vector_t b (1);
	b.setZero ();

	return boost::make_shared<
	  GenericNumericLinearFunction<typename T::traits_t> >
	  (A, b);
      }

      template <typename T>
      boost::shared_ptr<T>
      laplacianDeformationEnergy (const MarkerFunctionData&)
      {
	return boost::shared_ptr<T> ();
      }

      template <typename T>
      boost::shared_ptr<T>
      boneLength (const MarkerFunctionData&)
      {
	return boost::shared_ptr<T> ();
      }

    } // end of namespace detail.

    MarkerFunctionFactory::MarkerFunctionFactory (const MarkerFunctionData& data)
      : data_ (data)
    {}

    MarkerFunctionFactory::~MarkerFunctionFactory ()
    {}

    template <typename T>
    boost::shared_ptr<T>
    MarkerFunctionFactory::buildFunction (const std::string& name)
    {
      if (name == "null")
	return detail::null<T> (data_);
      else if (name == "lde")
	return detail::laplacianDeformationEnergy<T> (data_);
      else if (name == "bone-length")
	return detail::boneLength<T> (data_);
      throw std::runtime_error ("invalid function name");
    }

    template <typename T>
    Constraint<T>
    MarkerFunctionFactory::buildConstraint (const std::string& name)
    {
      Constraint<T> constraint;
      constraint.function = this->buildFunction<T> (name);
      return constraint;
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_MARKER_FUNCTION_FACTORY_HXX
