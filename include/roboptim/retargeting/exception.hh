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

#ifndef ROBOPTIM_RETARGETING_EXCEPTION_HH
# define ROBOPTIM_RETARGETING_EXCEPTION_HH
# include <iosfwd>
# include <stdexcept>

# include <roboptim/retargeting/config.hh>

/// \brief Declare a RobOptim Retargeting exception.
///
/// \warning You still have to provide the constructor and the
///	     destructor!
# define ROBOPTIM_RETARGETING_EXCEPTION(T)				\
  class ROBOPTIM_RETARGETING_DLLEXPORT T : public ::roboptim::retargeting::Exception \
  {									\
  public:								\
    T (const char* msg, const char* file, int line, const char* function); \
    virtual ~T () throw ();						\
  };									\
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

namespace roboptim
{
  namespace retargeting
  {
    /// \brief Top-level exception class for roboptim-retargeting.
    class ROBOPTIM_RETARGETING_DLLEXPORT Exception : public std::runtime_error
    {
    public:
      Exception (const char* msg,
		 const char* file,
		 int line,
		 const char* function); // no throw
      virtual ~Exception () throw ();

      std::ostream& print (std::ostream&) const;

    private:
      const char* file_;
      int line_;
      const char* function_;
    };

    ROBOPTIM_RETARGETING_DLLEXPORT std::ostream&
    operator<< (std::ostream&, const Exception&);

    ROBOPTIM_RETARGETING_EXCEPTION (Assertion);
    ROBOPTIM_RETARGETING_EXCEPTION (PreCondition);
    ROBOPTIM_RETARGETING_EXCEPTION (PostCondition);

    ROBOPTIM_RETARGETING_EXCEPTION (BadPointer);

    class ROBOPTIM_RETARGETING_DLLEXPORT MarkerNotFound : public Exception
    {
    public:
      MarkerNotFound
      (const std::string& markerName, const char* file, int line,
       const char* function);
      virtual ~MarkerNotFound () throw ();
    };


  } // end of namespace retargeting.
} // end of namespace roboptim.

# undef ROBOPTIM_RETARGETING_EXCEPTION
#endif //! ROBOPTIM_RETARGETING_EXCEPTION_HH
