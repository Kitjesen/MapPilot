
#include "allan_variance_ros2/plot_imu.hpp"

void plot_imu(std::string bag_path, std::string imu_topic) {
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Processing " << bag_path << " ...");

  uint64_t tCurrNanoSeconds_{};
  uint64_t lastImuTime_{};
  uint64_t firstTime_{};
  int skipped_imu_{};
  bool firstMsg_ = true;

  try {
    
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    // Prepare the filter on the selected IMU topics
    rosbag2_storage::StorageFilter storage_filter;
    storage_filter.topics.push_back(imu_topic);

    rosbag2_cpp::readers::SequentialReader bag;
    bag.open(storage_options, converter_options);
    bag.set_filter(storage_filter);

    std::vector<rosbag2_storage::TopicMetadata> available_topics = bag.get_all_topics_and_types();

    for (const auto available_topic : available_topics)
    {
      if (available_topic.name == imu_topic)
      {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Topic " << available_topic.name << " found in the bag!");
      }
    }

    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg;
    sensor_msgs::msg::Imu::SharedPtr imu9dof_msg = std::make_shared<sensor_msgs::msg::Imu>();
    rclcpp::Serialization<sensor_msgs::msg::Imu> serialization_;
    rerun::RecordingStream rec("allan_variance_plot_imu");  
    
    while (bag.has_next())
    {
      msg = bag.read_next();
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);   
      serialization_.deserialize_message(&serialized_msg, imu9dof_msg.get());

      tCurrNanoSeconds_ = imu9dof_msg->header.stamp.nanosec + imu9dof_msg->header.stamp.sec * 1e9;
      if (firstMsg_)
      {
        firstMsg_ = false;
        firstTime_ = tCurrNanoSeconds_;
        lastImuTime_ = tCurrNanoSeconds_;
      }

      if (tCurrNanoSeconds_ < lastImuTime_) {
        skipped_imu_++;
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"IMU out of order. Current(ns): "
                          << tCurrNanoSeconds_ - firstTime_ << " Last(ns): "
                          << lastImuTime_ - firstTime_ << " (" << skipped_imu_ << " dropped)");
        continue;
      }

      lastImuTime_ = tCurrNanoSeconds_;

      // Log data to the internal buffer.
      rec.set_time_nanos("header_timestamp", (int64_t)tCurrNanoSeconds_);
      rec.set_time_nanos("received_timestamp", msg->time_stamp);
      rec.log("acc/x", rerun::Scalar(imu9dof_msg->linear_acceleration.x));
    }
    rec.save((std::string)"rerun_file.rrd");

    if (!rclcpp::ok())
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Finished! Closing the bag...");
      bag.close();
      return;
    }
    }

    catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Captured unknown exception");
    }

}

int main(int argc, char** argv) {

  std::string bag_path;
  std::string topic_name;
  
  if (argc >= 2)
  {
    bag_path = argv[1];
    topic_name = argv[2];
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Bag Path = " << bag_path);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Topic = " << topic_name);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Rosbag folder and/or config file not provided!");
    return 0;
  }

  plot_imu(bag_path, topic_name);

  return 0;
}

