#include <boost/circular_buffer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

// CONSTRUCTOR
class VehicleEventRecorder : public rclcpp::Node
{
public:
  VehicleEventRecorder();

private:
  void handle_message(const std::string &topic_name, std::shared_ptr<rclcpp::SerializedMessage> msg);
  void on_crash_event(const std_msgs::msg::Bool::SharedPtr msg);
  void write_message_to_mcap(const std::string &topic_name, std::shared_ptr<rclcpp::SerializedMessage> msg, rclcpp::Time timestamp);

  struct BufferedMessage {
    rclcpp::Time timestamp;
    std::string topic_name;
    std::shared_ptr<rclcpp::SerializedMessage> message;

    BufferedMessage(const rclcpp::Time &ts, const std::string &tn, std::shared_ptr<rclcpp::SerializedMessage> msg)
        : timestamp(ts), topic_name(tn), message(msg) {}
  };

  bool is_recording_post_crash_;
  boost::circular_buffer<BufferedMessage> pre_crash_data_; // Circular buffer container
  std::shared_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> generic_subscriptions_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr crash_event_subscriber_;

};

// SETUP
VehicleEventRecorder::VehicleEventRecorder() : Node("vehicle_event_data_recorder"),
                                               is_recording_post_crash_(false),
                                               pre_crash_data_(100) // Buffer size for 100 messages
{
  // MCAP setup
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = "vehicle_event_data";
  storage_options.storage_id = "mcap";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  writer_ = std::make_shared<rosbag2_cpp::writers::SequentialWriter>();
  writer_->open(storage_options, converter_options);

  // IMPORTANT: spin briefly to let the discovery happen and see the topics
  rclcpp::Rate r(100);
  int i = 0;
  while (rclcpp::ok() && i < 100) {
    rclcpp::spin_some(this->get_node_base_interface());
    r.sleep();
    i++;
  }

  // Subscribe to all topics
  auto topics_and_types = this->get_topic_names_and_types();
  for (const auto &topic_and_type : topics_and_types)
  {
    auto topic_name = topic_and_type.first;
    if (topic_name == "/parameter_events" || topic_name == "/rosout" || topic_name == "/events/read_split") {
      RCLCPP_INFO(this->get_logger(), "Skipping subscription to: '%s'", topic_name.c_str());
      continue;
    }
    RCLCPP_INFO(this->get_logger(), "Detect topic: '%s'", topic_name.c_str());
    auto topic_type = topic_and_type.second[0]; // Assuming one type per topic

    //create topic before write
    rosbag2_storage::TopicMetadata topic_metadata;
    topic_metadata.name = topic_name;
    topic_metadata.type = topic_type;
    topic_metadata.serialization_format = "cdr";
    writer_->create_topic(topic_metadata);
    RCLCPP_INFO(this->get_logger(), "create bag write topic: '%s'", topic_name.c_str());

    //create subscription
    auto sub = this->create_generic_subscription(
        topic_name, topic_type, 10, [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          this->handle_message(topic_name, msg);
        });
    generic_subscriptions_.push_back(sub);
    RCLCPP_INFO(this->get_logger(), "Subscribe to: '%s'", topic_name.c_str());
  }

  // Subscribe to crash event topic
  crash_event_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      "/crash_event", 10, std::bind(&VehicleEventRecorder::on_crash_event, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribe to: /crash_event");
  // current /crash_event send: ros2 topic pub --once /crash_event std_msgs/msg/Bool "{data: true}"
}

void VehicleEventRecorder::handle_message(const std::string &topic_name, std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  auto timestamp = this->now();
  pre_crash_data_.push_back(BufferedMessage(timestamp, topic_name, msg)); // Adds message to circular buffer

  if (is_recording_post_crash_)
  {
    write_message_to_mcap(topic_name, msg, timestamp);
  }
}

void VehicleEventRecorder::on_crash_event(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data)
  {
    RCLCPP_INFO(this->get_logger(), "Crash detected. Writing pre-crash and post-crash data.");

    // Switch to post-crash recording
    is_recording_post_crash_ = true;
    
    // Write all pre-crash data from the buffer to MCAP
    for (const auto &data : pre_crash_data_)
    {
      write_message_to_mcap(data.topic_name, data.message, data.timestamp);
    }

    RCLCPP_INFO(this->get_logger(), "Buffer writing finish");

  }
}

void VehicleEventRecorder::write_message_to_mcap(const std::string &topic_name, std::shared_ptr<rclcpp::SerializedMessage> msg, rclcpp::Time timestamp) {

    auto serialized_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    serialized_msg->topic_name = topic_name;
    serialized_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>(msg->get_rcl_serialized_message());
    serialized_msg->time_stamp = timestamp.nanoseconds();
    writer_->write(serialized_msg);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleEventRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
