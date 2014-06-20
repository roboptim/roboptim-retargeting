// Copyright (C) 2013 by Thomas Moulard, AIST, CNRS, INRIA.
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

#define BOOST_TEST_MODULE choreonoid_body_motion

#include <sstream>
#include <fstream>

#include <boost/make_shared.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <roboptim/retargeting/function/choreonoid-body-trajectory.hh>
#include <roboptim/retargeting/io/choreonoid-body-motion.hh>

#include <cnoid/BodyMotion>

using namespace roboptim;
using namespace roboptim::retargeting;

using boost::test_tools::output_test_stream;

static void readWrite (const std::string& file, const std::string& output)
{
  // Loading the motion.
  cnoid::BodyMotionPtr bodyMotion = boost::make_shared<cnoid::BodyMotion> ();

  bodyMotion->loadStandardYAMLformat
    (file);

  // Building the trajectory.
  boost::shared_ptr<ChoreonoidBodyTrajectory> trajectory =
    boost::make_shared<ChoreonoidBodyTrajectory> (bodyMotion, true);

  writeBodyMotion (output, trajectory);
}

BOOST_AUTO_TEST_CASE (simple)
{
  // Loading the motion.
  std::string file = DATA_DIR;
  file += "/sample.body-motion.yaml";

  readWrite (file, "/tmp/test.yaml");
  readWrite ("/tmp/test.yaml", "/tmp/test2.yaml");

  // Check that the files are identical.
  //
  // With zsh, one could run:
  // md5sum =(head -n9 /tmp/test.yaml) =(head -n9 /tmp/test2.yaml)
  std::string str1;
  std::ifstream file1 ("/tmp/test.yaml");

  std::string str2;
  std::ifstream file2 ("/tmp/test2.yaml");

  // Errors on floating point values prevent from checking the whole
  // file so let's only look at the first 20 lines, it has been
  // empirically tested that it works for these values.
  for (int i = 0; i < 20; ++i)
    {
      std::getline (file1, str1) && std::getline (file2, str2);
      BOOST_CHECK_EQUAL (str1, str2);
    }
}
