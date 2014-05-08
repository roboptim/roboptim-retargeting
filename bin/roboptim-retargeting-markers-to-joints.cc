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

#include <string>

#include <boost/program_options.hpp>

#include <cnoid/Body>
#include <cnoid/BodyLoader>

#include <libmocap/marker-set.hh>
#include <libmocap/marker-set-factory.hh>
#include <libmocap/marker-trajectory.hh>
#include <libmocap/marker-trajectory-factory.hh>

#include <roboptim/trajectory/trajectory.hh>


namespace roboptim
{
  namespace retargeting
  {
    struct MarkerToJointOptions
    {
      std::string markerSet;
      std::string markersTrajectory;

      std::string outputFile;

      std::string robotModel;
    };

    struct MarkerToJointData
    {
      /// \brief marker set loaded by libmocap
      libmocap::MarkerSet markerSet;

      /// \brief markers trajectory as loaded by libmocap
      libmocap::MarkerTrajectory markersTrajectory;

      /// \brief Robot model loaded through Choreonoid
      ///
      /// Robot model contains the robot description of joints and
      /// bodies associated with limits such as joints positions,
      /// velocities limits, etc.
      cnoid::BodyPtr robotModel;

      /// \brief RobOptim markers trajectory
      boost::shared_ptr<roboptim::Trajectory<3> > inputTrajectory;

      /// \brief RobOPtim joints trajectory to be generated
      boost::shared_ptr<roboptim::Trajectory<3> > outputTrajectory;
    };

    void
    buildJointDataFromOptions (MarkerToJointData& data,
			       const MarkerToJointOptions& options);

    inline void
    buildJointDataFromOptions (MarkerToJointData& data,
			       const MarkerToJointOptions& options)
    {
      cnoid::BodyLoader loader;

      data.markerSet =
	libmocap::MarkerSetFactory ().load (options.markerSet);
      data.markersTrajectory =
	libmocap::MarkerTrajectoryFactory ().load (options.markersTrajectory);
      data.robotModel = loader.load (options.robotModel);
    }


  } // end of namespace retargeting
} // end of namespace roboptim

static bool parseOptions
(roboptim::retargeting::MarkerToJointOptions& options,
 int argc, const char* argv[])
{
  namespace po = boost::program_options;
  po::options_description desc ("Options");
  desc.add_options ()
    ("help,h", "Print help messages")
    ("markers-trajectory,m",
     po::value<std::string>
     (&options.markersTrajectory)->required (),
     "input markers trajectory used during Motion Capture"
     " (trc or any other format supported by libmocap)")
    ("output-file,o",
     po::value<std::string>
     (&options.outputFile)->required (),
     "output marker trajectory (Choreonoid YAML file)")

    ("marker-set,s",
     po::value<std::string> (&options.markerSet)->required (),
     "Marker Set used during Motion Capture"
     " (mars or any other format supported by libmocap)")

    ("robot-model,r",
     po::value<std::string> (&options.robotModel)->required (),
     "Robot Model (Choreonoid YAML file)")
    ;

  po::variables_map vm;
  po::store
    (po::command_line_parser (argc, argv)
     .options (desc)
     .run (),
     vm);

  if (vm.count ("help"))
    {
      std::cout << desc << "\n";
      return false;
    }

  po::notify (vm);

  return true;
}


int safeMain (int argc, const char* argv[])
{
  roboptim::retargeting::MarkerToJointOptions options;

  if (!parseOptions (options, argc, argv))
    return 0;

  roboptim::retargeting::MarkerToJointData data;
  buildJointDataFromOptions (data, options);

  //FIXME: for each frame apply IK

  return 0;
}


int main (int argc, const char* argv[])
{
  try
    {
      return safeMain (argc, argv);
    }
  catch (const std::exception& e)
    {
      std::cerr << e.what () << std::endl;
      return 1;
    }
}
