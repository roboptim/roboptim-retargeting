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

#define BOOST_TEST_MODULE morphing

#include <boost/test/unit_test.hpp>

#include <roboptim/retargeting/morphing.hh>

using namespace roboptim;
using namespace roboptim::retargeting;

BOOST_AUTO_TEST_CASE (morphing)
{
  std::string file = DATA_DIR;
  file += "/human-to-hrp4c.morphing.yaml";

  MorphingData morphing;
  morphing = loadMorphingData (file);

}
