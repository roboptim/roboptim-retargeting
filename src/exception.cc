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
#include <iostream>
#include <string>

#include <boost/format.hpp>

#include <roboptim/retargeting/exception.hh>

namespace roboptim
{
  namespace retargeting
  {
    Exception::Exception (const char* msg,
			  const char* file,
			  int line,
			  const char* function)
      : std::runtime_error (msg),
	file_ (file),
	line_ (line),
	function_ (function)
    {}

    Exception::~Exception () throw ()
    {}

    std::ostream&
    Exception::print (std::ostream& o) const
    {
      o << boost::format ("%s:%d:in %s:%s")
	% file_ % line_ % function_ % this->what ();
      return o;
    }

    std::ostream&
    operator<< (std::ostream& o, const Exception& e)
    {
      return e.print (o);
    }


    Assertion::Assertion
    (const char* msg, const char* file, int line, const char* function)
      : Exception (msg, file, line, function)
    {}

    Assertion::~Assertion () throw ()
    {}

    PreCondition::PreCondition
    (const char* msg, const char* file, int line, const char* function)
      : Exception (msg, file, line, function)
    {}

    PreCondition::~PreCondition () throw ()
    {}

    PostCondition::PostCondition
    (const char* msg, const char* file, int line, const char* function)
      : Exception (msg, file, line, function)
    {}

    PostCondition::~PostCondition () throw ()
    {}

    BadPointer::BadPointer
    (const char* msg, const char* file, int line, const char* function)
      : Exception (msg, file, line, function)
    {}

    BadPointer::~BadPointer () throw ()
    {}

    MarkerNotFound::MarkerNotFound
    (const std::string& markerName,
     const char* file, int line, const char* function)
      : Exception
	((boost::format ("marker `%s' not found") % markerName).str ().c_str (),
	 file, line, function)
    {}

    MarkerNotFound::~MarkerNotFound () throw ()
    {}
  } // end of namespace retargeting.
} // end of namespace roboptim.
