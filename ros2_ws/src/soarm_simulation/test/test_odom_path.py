from collections import deque
from unittest.mock import patch

from nav_msgs.msg import Odometry

from soarm_simulation.odom_path import OdomPath


class Publisher:
    def __init__(self, subscribers):
        self.subscribers = subscribers
        self.messages = []

    def get_subscription_count(self):
        return self.subscribers

    def publish(self, message):
        self.messages.append(message)


def node_for_test(subscribers, publish_interval=0.5):
    node = OdomPath.__new__(OdomPath)
    node.poses = deque(maxlen=2000)
    node.publisher = Publisher(subscribers)
    node.publish_interval = publish_interval
    node.last_publish_wall = None
    return node


def odometry(seconds):
    message = Odometry()
    message.header.stamp.sec = seconds
    message.header.frame_id = 'odom_combined'
    return message


def test_does_not_construct_or_publish_path_without_subscribers():
    node = node_for_test(0)
    node.on_odom(odometry(1))
    assert not node.publisher.messages
    assert len(node.poses) == 1
    assert node.last_publish_wall is None


def test_publishes_at_most_two_hz_with_latest_header_and_bounded_history():
    node = node_for_test(1)
    with patch('soarm_simulation.odom_path.time.monotonic', side_effect=[10.0, 10.2, 10.5]):
        node.on_odom(odometry(1))
        node.on_odom(odometry(2))
        node.on_odom(odometry(3))
    assert len(node.publisher.messages) == 2
    assert node.publisher.messages[0].header.stamp.sec == 1
    assert node.publisher.messages[1].header.stamp.sec == 3
    assert len(node.publisher.messages[1].poses) == 3
