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
# include <roboptim/retargeting/function/distance-to-marker.hh>

namespace roboptim
{
  namespace retargeting
  {
    namespace detail
    {
      template <typename T>
      boost::shared_ptr<T>
      null (const MarkerToJointFunctionData& data)
      {
	if (!data.outputTrajectoryReduced)
	  throw std::runtime_error
	    ("failed to create null function: no reduced joint trajectory");
	if (data.outputTrajectoryReduced->parameters ().size () == 0)
	  throw std::runtime_error
	    ("failed to create null function:"
	     " empty parameters vector in reduced joint trajectory");

	typename T::matrix_t A
	  (1, data.nDofsFiltered ());
	A.setZero ();
	typename T::vector_t b (1);
	b.setZero ();

	return boost::make_shared<
	  GenericNumericLinearFunction<typename T::traits_t> >
	  (A, b);
      }

      template <typename T>
      boost::shared_ptr<T>
      distanceToMarker (const MarkerToJointFunctionData& data)
      {
	boost::shared_ptr<
	  JointToMarkerPositionChoreonoid<typename T::traits_t> >
	  jointToMarker =
	  boost::make_shared<
	    JointToMarkerPositionChoreonoid<typename T::traits_t> >
	  (data.interactionMesh);

	//FIXME: frameId will always be 0, how to update?
	Function::vector_t referencePositions =
	  data.inputTrajectory->parameters ().segment
	  (data.frameId * jointToMarker->outputSize (),
	   jointToMarker->outputSize ());

	return boost::make_shared<
	  DistanceToMarker<typename T::traits_t> >
	  (jointToMarker, referencePositions);
      }

      template <typename T>
      boost::shared_ptr<T>
      jointsLimits (const MarkerToJointFunctionData& data)
      {
	typename T::matrix_t A
	  (data.nDofsFiltered (), data.nDofsFiltered ());
	A.setIdentity ();
	typename T::vector_t b (data.nDofsFiltered ());
	b.setZero ();
	return
	  boost::make_shared<
	    GenericNumericLinearFunction<typename T::traits_t> >
	  (A, b);
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
	{"distance-to-marker", &distanceToMarker<T>},
	{"joints-limits", &jointsLimits<T>},
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

    template <typename T>
    Constraint<T>
    MarkerToJointFunctionFactory::buildConstraint (const std::string& name)
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

      if (name == "joints-limits")
	{
	  std::size_t id = 0;
	  for (std::size_t jointId = 0;
	       jointId < static_cast<std::size_t> (data_.nDofsFull ()); ++jointId)
	    {
	      if (data_.disabledJointsConfiguration[jointId])
		{
		  int jointId_ = static_cast<int> (jointId);
		  if (jointId < 6)
		    constraint.intervals[id++] = Function::makeInfiniteInterval ();
		  else
		    constraint.intervals[id++] = Function::makeInterval
		      (data_.robotModel->joint (jointId_ - 6)->q_lower (),
		       data_.robotModel->joint (jointId_ - 6)->q_upper ());
		}
	    }
	}
      else
	throw std::runtime_error ("unknown constraint");
      return constraint;
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
