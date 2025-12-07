#!/usr/bin/env python3
"""
Reliability testing script for ROS 2 message exchange
Tests the reliability of publisher-subscriber communication
Used in the Physical AI & Humanoid Robotics educational examples
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading
from collections import deque
import statistics


class ReliabilityTestPublisher(Node):
    """
    Publisher node for reliability testing
    """
    def __init__(self, message_count=100, rate=10):
        super().__init__('reliability_test_publisher')
        self.publisher_ = self.create_publisher(String, 'reliability_test', 10)
        self.message_count = message_count
        self.rate = rate  # messages per second
        self.timer_period = 1.0 / rate
        self.current_msg_id = 0
        self.start_time = None

    def start_publishing(self):
        """
        Start the publishing timer
        """
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Publish a message with a unique ID
        """
        if self.current_msg_id < self.message_count:
            msg = String()
            msg.data = f'reliability_test_msg_{self.current_msg_id:04d}'
            self.publisher_.publish(msg)
            self.current_msg_id += 1
            self.get_logger().debug(f'Published: {msg.data}')
        else:
            # Stop the timer after all messages are sent
            self.timer.cancel()


class ReliabilityTestSubscriber(Node):
    """
    Subscriber node for reliability testing
    """
    def __init__(self):
        super().__init__('reliability_test_subscriber')
        self.subscription = self.create_subscription(
            String,
            'reliability_test',
            self.message_callback,
            10)
        self.received_messages = []
        self.expected_messages = set()
        self.start_time = None
        self.end_time = None

    def set_expected_count(self, count):
        """
        Set the expected number of messages
        """
        self.expected_messages = set(range(count))

    def message_callback(self, msg):
        """
        Process received messages
        """
        # Extract message ID from the format 'reliability_test_msg_XXXX'
        try:
            msg_id_str = msg.data.split('_')[-1]
            msg_id = int(msg_id_str)
            self.received_messages.append({
                'id': msg_id,
                'timestamp': self.get_clock().now(),
                'data': msg.data
            })
            self.get_logger().debug(f'Received: {msg.data}')

            # Remove from expected if present
            if msg_id in self.expected_messages:
                self.expected_messages.remove(msg_id)
        except (ValueError, IndexError):
            self.get_logger().warn(f'Could not parse message ID from: {msg.data}')

    def get_results(self):
        """
        Get reliability test results
        """
        total_sent = len(self.received_messages) + len(self.expected_messages)
        received_count = len(self.received_messages)
        missing_count = len(self.expected_messages)
        reliability_rate = (received_count / total_sent) * 100 if total_sent > 0 else 0

        # Calculate timing statistics if we have received messages
        if self.received_messages:
            timestamps = [msg['timestamp'] for msg in self.received_messages]
            time_diffs = [(timestamps[i+1].nanoseconds - timestamps[i].nanoseconds) / 1e9
                         for i in range(len(timestamps)-1)]
            avg_interval = statistics.mean(time_diffs) if time_diffs else 0
            interval_std = statistics.stdev(time_diffs) if len(time_diffs) > 1 else 0
        else:
            avg_interval = 0
            interval_std = 0

        return {
            'total_sent': total_sent,
            'received_count': received_count,
            'missing_count': missing_count,
            'reliability_rate': reliability_rate,
            'expected_messages': sorted(list(self.expected_messages)),
            'avg_interval': avg_interval,
            'interval_std': interval_std
        }


def run_reliability_test(message_count=100, rate=10):
    """
    Run the reliability test with specified parameters
    """
    print(f"Starting reliability test: {message_count} messages at {rate} Hz")
    print("="*60)

    # Initialize ROS 2
    rclpy.init()

    # Create publisher and subscriber
    publisher = ReliabilityTestPublisher(message_count, rate)
    subscriber = ReliabilityTestSubscriber()
    subscriber.set_expected_count(message_count)

    # Create an executor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(subscriber)

    # Start publishing
    publisher.start_publishing()

    # Calculate expected time (with buffer)
    expected_time = (message_count / rate) + 5  # 5 second buffer

    # Run the test
    try:
        start_time = time.time()
        executor.spin(timeout_sec=expected_time)
        elapsed_time = time.time() - start_time
    except KeyboardInterrupt:
        print("Test interrupted by user")
    finally:
        # Get results
        results = subscriber.get_results()

        # Print results
        print(f"\nReliability Test Results:")
        print(f"  Total messages sent: {results['total_sent']}")
        print(f"  Messages received: {results['received_count']}")
        print(f"  Messages missing: {results['missing_count']}")
        print(f"  Reliability rate: {results['reliability_rate']:.2f}%")
        print(f"  Average interval: {results['avg_interval']:.4f}s")
        print(f"  Interval std dev: {results['interval_std']:.4f}s")
        print(f"  Test duration: {elapsed_time:.2f}s")

        if results['expected_messages']:
            print(f"  Missing message IDs: {results['expected_messages'][:10]}{'...' if len(results['expected_messages']) > 10 else ''}")

        # Check if we met the 95% target
        success = results['reliability_rate'] >= 95
        print(f"\n  Test {'PASSED' if success else 'FAILED'}: {'✓' if success else '✗'}")
        print(f"  Target: ≥95% reliability, Actual: {results['reliability_rate']:.2f}%")

        # Clean up
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()
        executor.shutdown()

        return success, results


def main(args=None):
    """
    Main function to run the reliability test
    """
    print("ROS 2 Message Exchange Reliability Test")
    print("Target: 95% message delivery reliability")
    print()

    # Run test with default parameters
    success, results = run_reliability_test(message_count=100, rate=10)

    print("\n" + "="*60)
    if success:
        print("✓ RELIABILITY TEST PASSED - Message exchange meets 95% reliability target")
        return 0
    else:
        print("✗ RELIABILITY TEST FAILED - Message exchange does not meet 95% reliability target")
        return 1


if __name__ == '__main__':
    exit(main())