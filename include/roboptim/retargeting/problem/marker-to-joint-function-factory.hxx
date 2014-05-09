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

#ifndef ROBOPTIM_RETARGETING_JOINT_FUNCTION_FACTORY_HXX
# define  ROBOPTIM_RETARGETING_JOINT_FUNCTION_FACTORY_HXX
# include <stdexcept>

# include <roboptim/core/numeric-linear-function.hh>
# include <roboptim/core/filter/bind.hh>

# include <roboptim/retargeting/function/forward-geometry/choreonoid.hh>

namespace roboptim
{
  namespace retargeting
  {
    namespace detail
    {
      template <typename T>
      boost::shared_ptr<T>
      null (const MarkerToJointFunctionData&)
      {
	// if (!data.trajectory)
	//   throw std::runtime_error
	//     ("failed to create null function: no joint trajectory");
	// if (data.trajectory->parameters ().size () == 0)
	//   throw std::runtime_error
	//     ("failed to create null function:"
	//      " empty parameters vector in joint trajectory");

	// typename T::matrix_t A
	//   (1, data.trajectory->parameters ().size ());
	// A.setZero ();
	// typename T::vector_t b (1);
	// b.setZero ();

	// return boost::make_shared<
	//   GenericNumericLinearFunction<typename T::traits_t> >
	//   (A, b);
	return boost::shared_ptr<T> ();
      }

      template <typename T>
      boost::shared_ptr<T>
      forwardGeometry (const MarkerToJointFunctionData&)
      {
	return boost::shared_ptr<T> ();
      }

      /// \brief Map function name to the function used to allocate
      /// them.
      template <typename T>
      struct MarkerToJointFunctionFactoryMapping
      {
	/// \brief Pair (name, pointer to allocator function)
	struct Mapping
	{
	  const char* name;
	  boost::shared_ptr<T> (*factory) (const MarkerToJointFunctionData&);
	};

	/// \brief Static map between function names and their
	/// allocator function.
	static const Mapping map[];
      };

      template <typename T>
      const typename MarkerToJointFunctionFactoryMapping<T>::Mapping
      MarkerToJointFunctionFactoryMapping<T>::map[] = {
	{"null", &null<T>},
	{"forward-geometry", &forwardGeometry<T>},
	{0, 0}
      };
    } // end of namespace detail.

    MarkerToJointFunctionFactory::MarkerToJointFunctionFactory
    (const MarkerToJointFunctionData& data)
      : data_ (data)
    {}

    MarkerToJointFunctionFactory::~MarkerToJointFunctionFactory ()
    {}

    template <typename T>
    boost::shared_ptr<T>
    MarkerToJointFunctionFactory::buildFunction (const std::string& name)
    {
      const typename detail::MarkerToJointFunctionFactoryMapping<T>::Mapping* element =
	detail::MarkerToJointFunctionFactoryMapping<T>::map;

      while (element && element->name)
	{
	  if (element->name == name)
	    if (element->factory)
	      return (*element->factory) (data_);
	  element++;
	}
      throw std::runtime_error ("invalid function name");
    }

    inline std::vector<std::string>
    MarkerToJointFunctionFactory::listFunctions ()
    {
      std::vector<std::string> functions;

      const detail::MarkerToJointFunctionFactoryMapping<
	DifferentiableFunction>::Mapping*
	element =
	detail::MarkerToJointFunctionFactoryMapping<DifferentiableFunction>::map;
      while (element && element->name)
	{
	  functions.push_back (element->name);
	  element++;
	}
      return functions;
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_JOINT_FUNCTION_FACTORY_HXX
