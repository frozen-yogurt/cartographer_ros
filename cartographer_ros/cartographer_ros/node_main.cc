/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, false,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

DEFINE_double(external_x, 0.0,
"user specified robot location x");

DEFINE_double(external_y, 0.0,
"user specified robot location y");

DEFINE_double(external_theta, 0.0,
"user specified robot orientation");

namespace cartographer_ros {
namespace {

void Run(float external_x, float external_y, float external_theta) {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  auto map_builder =
      cartographer::common::make_unique<cartographer::mapping::MapBuilder>(
          node_options.map_builder_options);
  Node node(node_options, std::move(map_builder), &tf_buffer);
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);

    //int64_t timestamp = cartographer::common::ToUniversal(FromRos(ros::Time::now()));
    cartographer::transform::proto::Rigid3d relative_pose =
      cartographer::transform::ToProto(cartographer::transform::Rigid3d::Identity());
    // we only use the 2d transform, so just set x, y, and pitch for rotation
    relative_pose.mutable_translation()->set_x(-20.0);
    relative_pose.mutable_translation()->set_y(89.0);
    relative_pose.mutable_rotation()->set_z(sin(external_theta * 0.5));
    relative_pose.mutable_rotation()->set_w(cos(external_theta * 0.5));

    *trajectory_options.trajectory_builder_options
      .mutable_initial_trajectory_pose()
      ->mutable_relative_pose() = relative_pose;
    trajectory_options.trajectory_builder_options
      .mutable_initial_trajectory_pose()
      ->set_to_trajectory_id(0);
    // set the pose relative to the first node in the trajectory
    trajectory_options.trajectory_builder_options
      .mutable_initial_trajectory_pose()
      ->set_timestamp(0);

    LOG(INFO) << "xx "
              << trajectory_options.trajectory_builder_options.initial_trajectory_pose().ShortDebugString();
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  ::ros::spin();

  node.FinishAllTrajectories();
  node.RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run(FLAGS_external_x, FLAGS_external_y, FLAGS_external_theta);
  ::ros::shutdown();
}
