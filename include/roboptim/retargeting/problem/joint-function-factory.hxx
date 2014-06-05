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

# include <roboptim/retargeting/function/body-laplacian-deformation-energy/choreonoid.hh>
# include <roboptim/retargeting/function/forward-geometry/choreonoid.hh>
# include <roboptim/retargeting/function/torque/choreonoid.hh>
# include <roboptim/retargeting/function/zmp/choreonoid.hh>

namespace roboptim
{
  namespace retargeting
  {
    namespace detail
    {
      template <typename T>
      boost::shared_ptr<T>
      null (const JointFunctionData& data)
      {
	if (!data.trajectory)
	  throw std::runtime_error
	    ("failed to create null function: no joint trajectory");
	if (data.trajectory->parameters ().size () == 0)
	  throw std::runtime_error
	    ("failed to create null function:"
	     " empty parameters vector in joint trajectory");

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
      freeze (const JointFunctionData& data)
      {
	typename T::matrix_t A
	  (data.nDofsFiltered (), data.nParametersFiltered ());
	A.setZero ();
	A.block
	  (0, 0, data.nDofsFiltered (), data.nDofsFiltered ()).setIdentity ();

	// extract configuration from first frame
	typename T::vector_t b (data.nDofsFiltered ());
	b = data.filteredTrajectory->parameters ().segment (0, data.nDofsFiltered ());
	b *= -1;

	return boost::make_shared<
	  GenericNumericLinearFunction<typename T::traits_t> >
	  (A, b);
      }

      template <typename T>
      boost::shared_ptr<T>
      leftFoot (const JointFunctionData& data)
      {
	return
	  boost::make_shared<ForwardGeometryChoreonoid<typename T::traits_t> >
	  (data.robotModel, "L_ANKLE_R");
      }

      template <typename T>
      boost::shared_ptr<T>
      rightFoot (const JointFunctionData& data)
      {
	return
	  boost::make_shared<ForwardGeometryChoreonoid<typename T::traits_t> >
	  (data.robotModel, "R_ANKLE_R");
      }

      template <typename T>
      boost::shared_ptr<T>
      jointsLimits (const JointFunctionData& data)
      {
	typename T::matrix_t A
	  (data.nDofsFiltered () * 2, data.nDofsFiltered () * 2);
	A.setIdentity ();
	typename T::vector_t b (data.nDofsFiltered () * 2);
	b.setZero ();
	return
	  boost::make_shared<
	    GenericNumericLinearFunction<typename T::traits_t> >
	  (A, b);
      }

      template <typename T>
      boost::shared_ptr<T>
      laplacianDeformationEnergy (const JointFunctionData& data)
      {
	// the joint to marker function maps one particular robot
	// configuration to a vector containing all the markers
	// positions in this particular configuration
	typedef JointToMarkerPositionChoreonoid<typename T::traits_t>
	  jointToMarker_t;
	boost::shared_ptr<jointToMarker_t>
          jointToMarker =
          boost::make_shared<jointToMarker_t>
	  (data.robotModel, data.morphing);

	// create the cost function using the full trajectory
	boost::shared_ptr<T> cost =
	  boost::make_shared<BodyLaplacianDeformationEnergyChoreonoid<
            typename T::traits_t> >
	  (data.interactionMesh,
	   data.trajectory->parameters (),
	  jointToMarker);

        // bind the joints that must not be taken into account
	cost = bind (cost, data.disabledJointsTrajectory);

        return cost;
      }

      //FIXME: torque requires a select-by-id filter before being
      // inserted... (?)
      template <typename T>
      boost::shared_ptr<T>
      torque (const JointFunctionData& data)
      {
	throw std::runtime_error ("not supported yet");
	return
	  boost::make_shared<TorqueChoreonoid<typename T::traits_t> >
	  (data.robotModel);
      }

      template <typename T>
      boost::shared_ptr<T>
      zmp (const JointFunctionData& data)
      {
	return
	  boost::make_shared<ZMPChoreonoid<typename T::traits_t> >
	  (data.robotModel);
      }

      /// \brief Map function name to the function used to allocate
      /// them.
      template <typename T>
      struct JointFunctionFactoryMapping
      {
	/// \brief Pair (name, pointer to allocator function)
	struct Mapping
	{
	  const char* name;
	  boost::shared_ptr<T> (*factory) (const JointFunctionData&);
	};

	/// \brief Static map between function names and their
	/// allocator function.
	static const Mapping map[];
      };

      template <typename T>
      const typename JointFunctionFactoryMapping<T>::Mapping
      JointFunctionFactoryMapping<T>::map[] = {
	{"null", &null<T>},
	{"freeze", &freeze<T>},
	{"lde", &laplacianDeformationEnergy<T>},
	{"left-foot", &leftFoot},
	{"right-foot", &rightFoot},
	{"joint-limits", &jointsLimits},
	{"torque", &torque},
	{"zmp", &zmp},
	{0, 0}
      };
    } // end of namespace detail.

    JointFunctionFactory::JointFunctionFactory (const JointFunctionData& data)
      : data_ (data)
    {}

    JointFunctionFactory::~JointFunctionFactory ()
    {}

    template <typename T>
    boost::shared_ptr<T>
    JointFunctionFactory::buildFunction (const std::string& name)
    {
      const typename detail::JointFunctionFactoryMapping<T>::Mapping* element =
	detail::JointFunctionFactoryMapping<T>::map;

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
    JointFunctionFactory::buildConstraint (const std::string& name)
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

      if (name == "freeze")
	{
	  std::fill (constraint.intervals.begin (),
		     constraint.intervals.end (),
		     Function::makeInterval (0., 0.));
	}
      else if (name == "left-foot" || name == "right-foot")
	{
	  Function::vector_t position =
	    (*constraint.function)
	    (data_.filteredTrajectory->parameters ().segment
	     (0, data_.nDofsFiltered ()));
	  for (Function::vector_t::Index i = 0; i < position.size (); ++i)
	    constraint.intervals[static_cast<std::size_t> (i)] =
	      Function::makeInterval (position[i], position[i]);

	  constraint.type = Constraint<T>::CONSTRAINT_TYPE_PER_FRAME;
	  constraint.stateFunctionOrder = 0;
	}
      else if (name == "joints-limits")
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
		      (data_.robotModel->joint (jointId_ - 6)->dq_lower (),
		       data_.robotModel->joint (jointId_ - 6)->dq_upper ());
		}
	    }
	    constraint.type = Constraint<T>::CONSTRAINT_TYPE_PER_FRAME;
	    constraint.stateFunctionOrder = 1;
	}
      else if (name == "torque")
	{
	  //FIXME: this should be loaded from the outside.
	  constraint.intervals[0] = Function::makeInterval (-Function::infinity(), Function::infinity()); // FREE FLOATING X
	  constraint.intervals[1] = Function::makeInterval (-Function::infinity(), Function::infinity()); // FREE FLOATING Y
	  constraint.intervals[2] = Function::makeInterval (-Function::infinity(), Function::infinity()); // FREE FLOATING Z
	  constraint.intervals[3] = Function::makeInterval (-Function::infinity(), Function::infinity()); // FREE FLOATING ROLL
	  constraint.intervals[4] = Function::makeInterval (-Function::infinity(), Function::infinity()); // FREE FLOATING PITCH
	  constraint.intervals[5] = Function::makeInterval (-Function::infinity(), Function::infinity()); // FREE FLOATING YAW
	  constraint.intervals[6 + 0] = Function::makeInterval (-63.55, 63.55); // R_HIP_Y
	  constraint.intervals[6 + 1] = Function::makeInterval (-186.21, 186.21); // R_HIP_R
	  constraint.intervals[6 + 2] = Function::makeInterval (-95.18, 95.18); // R_HIP_P
	  constraint.intervals[6 + 3] = Function::makeInterval (-145.98, 145.98); // R_KNEE_P
	  constraint.intervals[6 + 4] = Function::makeInterval (-111.42, 111.42); // R_ANKLE_P
	  constraint.intervals[6 + 5] = Function::makeInterval (-75.11, 75.11); // R_ANKLE_R
	  constraint.intervals[6 + 6] = Function::makeInterval (-52.78, 52.78); // R_TOE_P
	  constraint.intervals[6 + 7] = Function::makeInterval (-186.21, 186.21); // L_HIP_R
	  constraint.intervals[6 + 8] = Function::makeInterval (-151.4, 151.4); // L_HIP_Y
	  constraint.intervals[6 + 9] = Function::makeInterval (-95.18, 95.18); // L_HIP_P
	  constraint.intervals[6 + 10] = Function::makeInterval (-145.98, 145.98); // L_KNEE_P
	  constraint.intervals[6 + 11] = Function::makeInterval (-111.42, 111.42); // L_ANKLE_P
	  constraint.intervals[6 + 12] = Function::makeInterval (-75.11, 75.11); // L_ANKLE_R
	  constraint.intervals[6 + 13] = Function::makeInterval (-52.78, 52.78); // L_TOE_P
	  constraint.intervals[6 + 14] = Function::makeInterval (-97.53, 97.53); // CHEST_P
	  constraint.intervals[6 + 15] = Function::makeInterval (-96.93, 96.93); // CHEST_R
	  constraint.intervals[6 + 16] = Function::makeInterval (-90.97, 90.97); // CHEST_Y
	  constraint.intervals[6 + 17] = Function::makeInterval (-17.59, 17.59); // NECK_Y
	  constraint.intervals[6 + 18] = Function::makeInterval (-17.59, 17.59); // NECK_R
	  constraint.intervals[6 + 19] = Function::makeInterval (-17.59, 17.59); // NECK_P
	  constraint.intervals[6 + 20] = Function::makeInterval (-5.26, 5.26); // EYEBROW_P
	  constraint.intervals[6 + 21] = Function::makeInterval (-0.71, 0.71); // EYELID_P
	  constraint.intervals[6 + 22] = Function::makeInterval (-0.84, 0.84); // EYE_P
	  constraint.intervals[6 + 23] = Function::makeInterval (-0.42, 0.42); // EYE_Y
	  constraint.intervals[6 + 24] = Function::makeInterval (-4.72, 4.72); // MOUTH_P
	  constraint.intervals[6 + 25] = Function::makeInterval (-0.22, 0.22); // LOWERLIP_P
	  constraint.intervals[6 + 26] = Function::makeInterval (-0.29, 0.29); // UPPERLIP_P
	  constraint.intervals[6 + 27] = Function::makeInterval (-5.9, 5.9); // CHEEK_P
	  constraint.intervals[6 + 28] = Function::makeInterval (-181.74, 181.74); // R_SHOULDER_P
	  constraint.intervals[6 + 29] = Function::makeInterval (-62.83, 62.83); // R_SHOULDER_R
	  constraint.intervals[6 + 30] = Function::makeInterval (-20.47, 20.47); // R_SHOULDER_Y
	  constraint.intervals[6 + 31] = Function::makeInterval (-54.46, 54.46); // R_ELBOW_P
	  constraint.intervals[6 + 32] = Function::makeInterval (-6.33, 6.33); // R_WRIST_Y
	  constraint.intervals[6 + 33] = Function::makeInterval (-6.33, 6.33); // R_WRIST_R
	  constraint.intervals[6 + 34] = Function::makeInterval (-0.77, 0.77); // R_HAND_J0
	  constraint.intervals[6 + 35] = Function::makeInterval (-1.16, 1.16); // R_HAND_J1
	  constraint.intervals[6 + 36] = Function::makeInterval (-181.74, 181.74); // L_SHOULDER_P
	  constraint.intervals[6 + 37] = Function::makeInterval (-62.83, 62.83); // L_SHOULDER_R
	  constraint.intervals[6 + 38] = Function::makeInterval (-20.47, 20.47); // L_SHOULDER_Y
	  constraint.intervals[6 + 39] = Function::makeInterval (-54.46, 54.46); // L_ELBOW_P
	  constraint.intervals[6 + 40] = Function::makeInterval (-6.33, 6.33); // L_WRIST_Y
	  constraint.intervals[6 + 41] = Function::makeInterval (-6.33, 6.33); // L_WRIST_R
	  constraint.intervals[6 + 42] = Function::makeInterval (-0.77, 0.77); // L_HAND_J0
	  constraint.intervals[6 + 43] = Function::makeInterval (-1.16, 1.16); // L_HAND_J1

	  constraint.type = Constraint<T>::CONSTRAINT_TYPE_PER_FRAME;
	  constraint.stateFunctionOrder = 2;
	}
      else if (name == "zmp")
	{
	  Function::value_type soleX = 0.03;
	  Function::value_type soleY = 0.;
	  Function::value_type soleLength = 0.2; //FIXME:
	  Function::value_type soleWidth = 0.1; //FIXME:

	  constraint.intervals[0] =
	    Function::makeInterval
	    (soleX - .5 * soleLength,
	     soleX + .5 * soleLength);
	  constraint.intervals[1] =
	    Function::makeInterval
	    (soleY - .5 * soleWidth,
	     soleY + .5 * soleWidth);

	  constraint.type = Constraint<T>::CONSTRAINT_TYPE_PER_FRAME;
	  constraint.stateFunctionOrder = 2;
	}
      else
	throw std::runtime_error ("unknown constraint");

      return constraint;
    }

    inline std::vector<std::string>
    JointFunctionFactory::listFunctions ()
    {
      std::vector<std::string> functions;

      const detail::JointFunctionFactoryMapping<
	DifferentiableFunction>::Mapping*
	element =
	detail::JointFunctionFactoryMapping<DifferentiableFunction>::map;
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
