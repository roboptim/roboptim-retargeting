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

# include <roboptim/core/function/constant.hh>

# include <roboptim/retargeting/function/body-laplacian-deformation-energy/choreonoid.hh>

namespace roboptim
{
  namespace retargeting
  {
    namespace detail
    {
      template <typename T>
      boost::shared_ptr<T>
      null (const JointFunctionData&)
      {
	return boost::shared_ptr<T> ();
      }

      template <typename T>
      boost::shared_ptr<T>
      feet (const JointFunctionData&)
      {
	return boost::shared_ptr<T> ();
      }

      template <typename T>
      boost::shared_ptr<T>
      jointsLimits (const JointFunctionData&)
      {
	return boost::shared_ptr<T> ();
      }

      template <typename T>
      boost::shared_ptr<T>
      laplacianDeformationEnergy (const JointFunctionData&)
      {
	return boost::shared_ptr<T> ();
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
      else if (name == "feet")
	return detail::feet<T> (data_);
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
