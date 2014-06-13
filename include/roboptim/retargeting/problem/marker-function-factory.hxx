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
# include <algorithm>
# include <stdexcept>

# include <roboptim/core/numeric-linear-function.hh>
# include <roboptim/core/filter/plus.hh>

# include <roboptim/retargeting/function/marker-laplacian-deformation-energy/choreonoid.hh>
# include <roboptim/retargeting/function/bone-length.hh>

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
      laplacianDeformationEnergy (const MarkerFunctionData& data)
      {
	return boost::make_shared<MarkerLaplacianDeformationEnergyChoreonoidDense>
	  (data.mapping, data.mesh, data.trajectory);
      }

      template <typename T>
      boost::shared_ptr<T>
      boneLength (const MarkerFunctionData& data)
      {
	typedef BoneLengthError<typename T::traits_t> fun_t;
	typedef boost::shared_ptr<fun_t> funPtr_t;

	typedef MorphingData::mapping_t::const_iterator
	  body_const_iterator;
	typedef MorphingData::mappingData_t::const_iterator
	  marker_const_iterator;

	boost::shared_ptr<T> result;

	for (body_const_iterator it = data.morphing.mapping.begin ();
	     it != data.morphing.mapping.end (); ++it)
	  {
	    for (marker_const_iterator itMarker = it->second.begin ();
		 itMarker != it->second.end (); ++itMarker)
	      {
		for (marker_const_iterator itMarker2 = it->second.begin ();
		     itMarker2 != it->second.end (); ++itMarker2)
		  {
		    // Avoid duplicating markers by adding both
		    // (p1,p2) and (p2,p1). Make sure that p2 < p1
		    // Also never add (p,p).
		    if (itMarker2 - it->second.begin ()
			>= itMarker - it->second.begin ())
		      continue;

		    boost::shared_ptr<T> fun =
		      boost::make_shared<fun_t>
		      (itMarker->marker, itMarker2->marker, it->first,
		       data.markersTrajectory, data.robotModel);

		    if (!result)
		      result = fun;
		    else
		      result = result + fun;
		  }
	      }
	  }

	return result;
      }

      /// \brief Map function name to the function used to allocate
      /// them.
      template <typename T>
      struct MarkerFunctionFactoryMapping
      {
	/// \brief Pair (name, pointer to allocator function)
	struct Mapping
	{
	  const char* name;
	  boost::shared_ptr<T> (*factory) (const MarkerFunctionData&);
	};

	/// \brief Static map between function names and their
	/// allocator function.
	static const Mapping map[];
      };

      template <typename T>
      const typename MarkerFunctionFactoryMapping<T>::Mapping
      MarkerFunctionFactoryMapping<T>::map[] = {
	{"null", &null<T>},
	{"lde", &laplacianDeformationEnergy<T>},
	{"bone-length", &boneLength<T>},
	{0, 0}
      };

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
      const typename detail::MarkerFunctionFactoryMapping<T>::Mapping* element =
	detail::MarkerFunctionFactoryMapping<T>::map;

      while (element && element->name)
	{
	  if (element->name == name)
	    if (element->factory)
	      return (*element->factory) (data_);
	  element++;
	}
      throw std::runtime_error ("invalid function name");
    }

    template <typename T>
    Constraint<T>
    MarkerFunctionFactory::buildConstraint (const std::string& name)
    {
      Constraint<T> constraint;
      constraint.function = this->buildFunction<T> (name);

      constraint.intervals.resize
	(static_cast<std::size_t> (constraint.function->outputSize ()),
	 Function::makeInfiniteInterval ());
      constraint.scales.resize
	(static_cast<std::size_t> (constraint.function->outputSize ()),
	 1.);
      constraint.type = Constraint<T>::CONSTRAINT_TYPE_ONCE;
      constraint.stateFunctionOrder = 0;

      if (name == "bone-length")
	{
	  constraint.type = Constraint<T>::CONSTRAINT_TYPE_PER_FRAME;
	  std::fill (constraint.intervals.begin (),
		     constraint.intervals.end (),
		     Function::makeInterval (0., 0.));
	}
      else
	throw std::runtime_error ("unknown constraint");

      return constraint;
    }

    inline std::vector<std::string>
    MarkerFunctionFactory::listFunctions ()
    {
      std::vector<std::string> functions;

      const detail::MarkerFunctionFactoryMapping<
	DifferentiableFunction>::Mapping*
	element =
	detail::MarkerFunctionFactoryMapping<DifferentiableFunction>::map;
      while (element && element->name)
	{
	  functions.push_back (element->name);
	  element++;
	}
      return functions;
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_MARKER_FUNCTION_FACTORY_HXX
