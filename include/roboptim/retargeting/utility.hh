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

#ifndef ROBOPTIM_RETARGETING_UTILITY_HH
# define ROBOPTIM_RETARGETING_UTILITY_HH
# include <cassert>
# include <boost/shared_ptr.hpp>

# include <roboptim/core/indent.hh>
# include <roboptim/retargeting/exception.hh>

// Create two pseudo-functions called roboptim_likely and
// roboptim_unlikely.
//
// These functions help the compilation to predict the branching more
// accurately.
# ifdef __GNUC__
#  define roboptim_likely(x)       __builtin_expect (!! (x), 1)
#  define roboptim_unlikely(x)     __builtin_expect (!! (x), 0)
# else
#  define roboptim_likely(x)       (x)
#  define roboptim_unlikely(x)     (x)
# endif // __GNUC__

// Define a portable macro to access current function name.
//
// This is currently working on GCC and replaced with "unknown" when
// another compiler is used.
# ifdef __GNUC__
#  define ROBOPTIM_RETARGETING_FUNCTION __PRETTY_FUNCTION__
# else
#  define ROBOPTIM_RETARGETING_FUNCTION "unknown"
# endif // __GNUC__

# define ROBOPTIM_RETARGETING_LVALUE_ACCESSOR(ATTR, TYPE)	\
  const TYPE& ATTR () const					\
  {								\
    return ATTR##_;						\
  }								\
  struct e_n_d__w_i_t_h__s_e_m_i__c_o_l_o_n

# define ROBOPTIM_RETARGETING_RVALUE_ACCESSOR(ATTR, TYPE)	\
  TYPE& ATTR ()							\
  {								\
    return ATTR##_;						\
  }								\
  struct e_n_d__w_i_t_h__s_e_m_i__c_o_l_o_n

# define ROBOPTIM_RETARGETING_ACCESSOR(ATTR, TYPE)		\
  ROBOPTIM_RETARGETING_LVALUE_ACCESSOR (ATTR, TYPE);		\
  ROBOPTIM_RETARGETING_RVALUE_ACCESSOR (ATTR, TYPE);		\
  struct e_n_d__w_i_t_h__s_e_m_i__c_o_l_o_n


/// \brief This macro declares the class T as well as its associated
///        pointer type.
# define ROBOPTIM_RETARGETING_PREDECLARE_CLASS(T)	\
  class T;						\
  typedef boost::shared_ptr<T> T##ShPtr;		\
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

/// \brief Pre-declare templated RobOptim function, dense variant,
///	   sparse variant and associated pointers types.
# define ROBOPTIM_RETARGETING_PREDECLARE_FUNCTION_TPL(CLASS)		\
    template <typename T>						\
    class CLASS;							\
    typedef CLASS< ::roboptim::EigenMatrixDense> CLASS##Dense;		\
    typedef CLASS< ::roboptim::EigenMatrixSparse> CLASS##Sparse;	\
    typedef boost::shared_ptr<CLASS##Dense> CLASS##Dense##ShPtr;	\
    typedef boost::shared_ptr<CLASS##Dense> CLASS##Dense##ShPtr;	\
    struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n


/// \brief Smart assertion, assert in debug, throw in release.
# define ROBOPTIM_RETARGETING_CHECK(T,x)				\
  do {									\
    if (roboptim_unlikely (!(x)))					\
      {									\
	T exception (#x,__FILE__, __LINE__, ROBOPTIM_RETARGETING_FUNCTION); \
	std::cerr							\
	  << ::roboptim::incindent << "assertion failed:"		\
	  << ::roboptim::iendl						\
	  << exception << ::roboptim::decindent				\
	  << ::roboptim::iendl;						\
        assert ((x) && #x);						\
	throw T (#x,__FILE__, __LINE__, ROBOPTIM_RETARGETING_FUNCTION);	\
      }									\
  } while (0)

/// \brief Assert if x is not true.
# define ROBOPTIM_RETARGETING_ASSERT(x)					\
  ROBOPTIM_RETARGETING_CHECK (::roboptim::retargeting::Assertion, (x))

/// \brief Check a pre-condition.
# define ROBOPTIM_RETARGETING_PRECONDITION(x)				\
  ROBOPTIM_RETARGETING_CHECK (::roboptim::retargeting::PreCondition, (x))

/// \brief Check a post-condition.
# define ROBOPTIM_RETARGETING_POSTCONDITION(x)				\
  ROBOPTIM_RETARGETING_CHECK (::roboptim::retargeting::PostCondition, (x))

/// \brief If shared pointer is null, throw otherwise dereference.
template <typename T>
typename boost::shared_ptr<T>::reference safeGet (boost::shared_ptr<T> ptr)
{
  if (roboptim_unlikely (!ptr))
    throw ::roboptim::retargeting::BadPointer
      ("safeGet failed", __FILE__, __LINE__, ROBOPTIM_RETARGETING_FUNCTION);
  return *ptr;
}


// Declare useful typedefs and misc useful functions here.
# include <roboptim/trajectory/trajectory.hh>
namespace roboptim
{
  namespace retargeting
  {
    typedef ::roboptim::Trajectory<3> Trajectory;
    typedef boost::shared_ptr<Trajectory> TrajectoryShPtr;

    inline std::size_t
    numberOfDiscretizationPoints (const TrajectoryShPtr trajectory)
    {
      return static_cast<std::size_t>
	(safeGet (trajectory).parameters ().size ()
	 / safeGet (trajectory).outputSize ());
    }

  } // end of namespace retargeting
} // end of namespace roboptim

#endif //! ROBOPTIM_RETARGETING_UTILITY_HH
