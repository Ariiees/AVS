# use ros2 humble SimpleBagRecorder as a base code.
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import *
import rosbag2_py
import time
import threading
import sensor_msgs.msg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Benchmarker(Node):
    def __init__(self):
        super().__init__('benchmarker')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('db_name', 'lidar_bag'),
                ('db_type', 'sqlite3'),
                ('recorder_topic_name', 'test/lidar'),
                ('data_type', 'sensor_msgs/msg/PointCloud2'),
                ('subscription_topic_name', '/sensing/lidar/top/pointcloud_raw'),
                ('data_freq', 0.1),
                ('qos_depth', 10),
                ('qos_reliability', 'reliable'),
                ('qos_history', 'keep_last'),
                ('recording_time', 30)
            ])

        db_name = 'results/' + self.get_parameter('db_name').get_parameter_value().string_value
        db_type = self.get_parameter('db_type').get_parameter_value().string_value
        recorder_topic_name = self.get_parameter('recorder_topic_name').get_parameter_value().string_value
        data_type = self.get_parameter('data_type').get_parameter_value().string_value
        subscription_topic_name = self.get_parameter('subscription_topic_name').get_parameter_value().string_value
        self.expected_interval = self.get_parameter('data_freq').get_parameter_value().double_value
        qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value
        qos_reliability = self.get_parameter('qos_reliability').get_parameter_value().string_value
        qos_history = self.get_parameter('qos_history').get_parameter_value().string_value
        recording_time = self.get_parameter('recording_time').get_parameter_value().integer_value

        # write and storage setup
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri= db_name,
            storage_id= db_type)
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name= recorder_topic_name,
            type= 'sensor_msgs/msg/' + data_type,
            serialization_format='cdr')
        self.writer.create_topic(topic_info)


        # subscription setup
        reliability = QoSReliabilityPolicy.RELIABLE if qos_reliability == 'reliable' else QoSReliabilityPolicy.BEST_EFFORT
        history = QoSHistoryPolicy.KEEP_LAST if qos_history == 'keep_last' else QoSHistoryPolicy.KEEP_ALL

        message_class = getattr(sensor_msgs.msg, data_type)

        qos_profile = QoSProfile(
            reliability=reliability,
            history=history,
            depth=qos_depth
        )

        self.subscription = self.create_subscription(
            message_class,
            subscription_topic_name,
            self.topic_callback,
            qos_profile)
        
        self.subscription

        # Initialize variables
        self.data_loss = 0
        self.last_msg_time = None

        # Open files for recording
        self.latency_file = open('results/latency_' + db_type + '.txt', 'w')
        self.throughput_file = open('results/throughput_'+ db_type + '.txt', 'w')
        self.data_loss_file = open('results/data_loss_' + db_type + '.txt', 'w')

        # Start the 30-second timer
        self.timer_thread = threading.Timer(recording_time, self.stop_recording)
        self.timer_thread.start()


    def topic_callback(self, msg):
        receive_time = time.time()
        
        # write
        self.writer.write(
            'test/lidar',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
        
        # Latency
        latency = time.time() - receive_time
        # current_msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # communicate_latency = time.time() - current_msg_time
        self.get_logger().info(f'Latency: {latency:.6f} seconds')
        # self.get_logger().info(f'Latency including communication: {communicate_latency:.6f} seconds')

        # Data loss
        current_msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_msg_time is not None:
            actual_interval = current_msg_time - self.last_msg_time
            if actual_interval > self.expected_interval:
                expected_messages = int(actual_interval / self.expected_interval)
                if expected_messages > 1:
                    self.data_loss = expected_messages - 1
                    self.get_logger().warn(f'Data loss detected: {self.data_loss} messages lost')
                    self.data_loss = 0
        self.last_msg_time = current_msg_time

        # Throughput
        bytes_written = len(serialize_message(msg)) / 1024
        throughput = bytes_written/latency
        self.get_logger().info(f'Storing Throughput: {throughput:.2f} bytes/second')

        # Try to write resulte to .txt
        try:
            self.latency_file.write(f'{latency:.6f}\n')
            self.data_loss_file.write(f'{self.data_loss}\n')
            self.throughput_file.write(f'{throughput:.2f}\n')
        except ValueError as e:
            self.get_logger().error(f'Error writing txt file: {e}')

    def stop_recording(self):
        self.get_logger().info('Stopping recording after 30 seconds.')
        self.unsubscribe()
        self.destroy_node()  # Close files and stop the node
        rclpy.shutdown()

    def destroy_node(self):
        # Close files when shutting down
        try:
            self.latency_file.close()
            self.throughput_file.close()
            self.data_loss_file.close()
        except IOError as e:
            self.get_logger().error(f'Error closing files: {e}')
        finally:
            super().destroy_node()

    def unsubscribe(self):
        if self.subscription:
            self.get_logger().info('Unsubscribing from topic.')
            self.destroy_subscription(self.subscription)
            self.subscription = None


def main(args=None):
    rclpy.init(args=args)
    sbr = Benchmarker()
    rclpy.spin(sbr)


if __name__ == '__main__':
    main()