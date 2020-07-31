#include <thread>
#include <chrono>

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/exceptions.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "std_srvs/Empty.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"

DEFINE_string(bag_filename, "",
    "If non-empty, filename of a .bag file to load, containing sensor data.");

int main(int argc, char** argv) {
  // TODO: need to check if cartographer node exist
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_bag_filename.empty()) << "-bag_filename is missing.";
  cartographer_ros::ScopedRosLogSink ros_log_sink;

  ::ros::init(argc, argv, "cartographer_node_finish_monitor");
  std_srvs::Empty srv;
  rosbag::Bag bag;
  ros::NodeHandle n;
  ros::ServiceClient client = 
    n.serviceClient<std_srvs::Empty>(cartographer_ros::kShutDownServiceName);
  
  try {
    LOG(INFO) << "Opening bag file " << FLAGS_bag_filename;
    bag.open(FLAGS_bag_filename, rosbag::bagmode::Read);
    rosbag::View view(bag);
    const double duration_in_seconds = (view.getEndTime() - view.getBeginTime()).toSec();
    bag.close();
    LOG(INFO) << FLAGS_bag_filename << " duration is " << duration_in_seconds << " sec.";
    std::chrono::seconds duration((int)duration_in_seconds + cartographer_ros::kSignalFinishDelaySecond);
    std::this_thread::sleep_for(duration);
    LOG(INFO) << "Timeout reached, sending finish signal to cartographer_node...";
    if(!client.call(srv)) {
      LOG(WARNING) << "Cartographer_node failed receiving signal";
      return 0;
    }
    LOG(INFO) << "Cartographer_node received signal";
  } catch (const rosbag::BagException& ex) {
    LOG(ERROR) << ex.what();
    return -1;
  }
  
  return 0;
}