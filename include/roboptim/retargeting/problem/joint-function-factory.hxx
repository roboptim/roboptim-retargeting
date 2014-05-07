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

	typename T::vector_t b (data.nDofsFiltered ());
	b.setZero ();

	return boost::make_shared<
	  GenericNumericLinearFunction<typename T::traits_t> >
	  (A, b);
      }

      template <typename T>
      boost::shared_ptr<T>
      leftFoot (const JointFunctionData& data)
      {
	return boost::make_shared<ForwardGeometryChoreonoid<typename T::traits_t> >
	  (data.robotModel, "L_ANKLE_R");
      }

      template <typename T>
      boost::shared_ptr<T>
      rightFoot (const JointFunctionData& data)
      {
	return boost::make_shared<ForwardGeometryChoreonoid<typename T::traits_t> >
	  (data.robotModel, "R_ANKLE_R");
      }

      template <typename T>
      boost::shared_ptr<T>
      jointsLimits (const JointFunctionData& data)
      {
	typename T::matrix_t A
	  (data.nDofsFiltered () * 2, data.nParametersFiltered ());
	A.setZero ();
	A.block
	  (0, 0, data.nDofsFiltered (), data.nDofsFiltered ()).setIdentity ();
	A.block
	  (0, data.nDofsFiltered (),
	   data.nDofsFiltered (), data.nDofsFiltered ()).setIdentity ();

	typename T::vector_t b (data.nDofsFiltered ());
	b.setZero ();

	return boost::make_shared<
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
	  (data.interactionMesh);

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

      template <typename T>
      boost::shared_ptr<T>
      torque (const JointFunctionData&)
      {
	return boost::shared_ptr<T> ();
      }

      template <typename T>
      boost::shared_ptr<T>
      zmp (const JointFunctionData&)
      {
	return boost::shared_ptr<T> ();
      }

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
      if (name == "null")
	return detail::null<T> (data_);
      else if (name == "lde")
	return detail::laplacianDeformationEnergy<T> (data_);
      else if (name == "left-foot")
	return detail::leftFoot<T> (data_);
      else if (name == "right-foot")
	return detail::rightFoot<T> (data_);
      else if (name == "joints-limits")
	return detail::jointsLimits<T> (data_);
      else if (name == "torque")
	return detail::torque<T> (data_);
      else if (name == "zmp")
	return detail::zmp<T> (data_);
      throw std::runtime_error ("invalid function name");
    }

    template <typename T>
    Constraint<T>
    JointFunctionFactory::buildConstraint (const std::string& name)
    {
      Constraint<T> constraint;
      constraint.function = this->buildFunction<T> (name);
      return constraint;
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_JOINT_FUNCTION_FACTORY_HXX
