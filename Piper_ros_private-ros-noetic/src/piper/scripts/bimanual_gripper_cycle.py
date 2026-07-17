#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import copy
import threading

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


DEFAULT_JOINT_NAMES = [
    'joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
]


class JointStateCache:
    def __init__(self, topic):
        self.topic = topic
        self._msg = None
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._sub = rospy.Subscriber(topic, JointState, self._callback, queue_size=1, tcp_nodelay=True)

    def _callback(self, msg):
        with self._lock:
            self._msg = msg
        self._event.set()

    def wait(self, timeout):
        if not self._event.wait(timeout):
            raise rospy.ROSException(f'timeout waiting for topic: {self.topic}')
        return self.get()

    def get(self):
        with self._lock:
            if self._msg is None:
                return None
            return copy.deepcopy(self._msg)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def build_joint_command(src_msg, target_gripper):
    if src_msg is None:
        raise rospy.ROSException('joint state not available')
    if len(src_msg.position) < 7:
        raise rospy.ROSException(f'joint state on {src_msg._connection_header.get("topic", "unknown")} has fewer than 7 joints')

    cmd = JointState()
    cmd.header.stamp = rospy.Time.now()
    cmd.name = list(src_msg.name) if src_msg.name else list(DEFAULT_JOINT_NAMES)
    cmd.position = list(src_msg.position[:7])
    cmd.position[6] = clamp(target_gripper, 0.0, 0.08)
    return cmd


def publish_pair(left_pub, right_pub, left_cache, right_cache, left_gripper, right_gripper):
    left_state = left_cache.get()
    right_state = right_cache.get()
    left_msg = build_joint_command(left_state, left_gripper)
    right_msg = build_joint_command(right_state, right_gripper)
    left_pub.publish(left_msg)
    right_pub.publish(right_msg)


def move_grippers(left_pub, right_pub, left_cache, right_cache, left_target, right_target, move_time, rate_hz):
    left_state = left_cache.get()
    right_state = right_cache.get()
    if left_state is None or right_state is None:
        raise rospy.ROSException('missing current joint states before moving grippers')

    left_start = float(left_state.position[6])
    right_start = float(right_state.position[6])
    steps = max(1, int(move_time * rate_hz))
    rate = rospy.Rate(rate_hz)

    for step in range(1, steps + 1):
        alpha = float(step) / float(steps)
        left_cmd = left_start + (left_target - left_start) * alpha
        right_cmd = right_start + (right_target - right_start) * alpha
        publish_pair(left_pub, right_pub, left_cache, right_cache, left_cmd, right_cmd)
        rate.sleep()

    for _ in range(5):
        publish_pair(left_pub, right_pub, left_cache, right_cache, left_target, right_target)
        rate.sleep()


def main():
    parser = argparse.ArgumentParser(description='Cycle left and right Piper grippers open/close.')
    parser.add_argument('--cycles', type=int, default=2, help='Number of open-close cycles.')
    parser.add_argument('--open', dest='open_pos', type=float, default=0.08, help='Open gripper position in meters.')
    parser.add_argument('--close', dest='close_pos', type=float, default=0.0, help='Close gripper position in meters.')
    parser.add_argument('--move-time', type=float, default=1.0, help='Seconds for each open/close move.')
    parser.add_argument('--pause-time', type=float, default=0.5, help='Seconds to wait after each move.')
    parser.add_argument('--rate', type=float, default=50.0, help='Publish rate during interpolation.')
    parser.add_argument('--state-left-topic', type=str, default='/puppet/joint_left', help='Left arm state topic.')
    parser.add_argument('--state-right-topic', type=str, default='/puppet/joint_right', help='Right arm state topic.')
    parser.add_argument('--cmd-left-topic', type=str, default='/master/joint_left', help='Left arm command topic.')
    parser.add_argument('--cmd-right-topic', type=str, default='/master/joint_right', help='Right arm command topic.')
    parser.add_argument('--enable-topic', type=str, default='/enable_flag', help='Enable topic used by piper_start_ms_node.')
    parser.add_argument('--send-enable', action='store_true', help='Publish True to the enable topic before running.')
    parser.add_argument('--wait-timeout', type=float, default=5.0, help='Seconds to wait for initial joint states.')
    args = parser.parse_args()

    rospy.init_node('bimanual_gripper_cycle', anonymous=True)

    left_cache = JointStateCache(args.state_left_topic)
    right_cache = JointStateCache(args.state_right_topic)

    left_pub = rospy.Publisher(args.cmd_left_topic, JointState, queue_size=1, tcp_nodelay=True)
    right_pub = rospy.Publisher(args.cmd_right_topic, JointState, queue_size=1, tcp_nodelay=True)
    enable_pub = rospy.Publisher(args.enable_topic, Bool, queue_size=1, latch=True)

    rospy.loginfo('waiting for current left/right joint states...')
    left_cache.wait(args.wait_timeout)
    right_cache.wait(args.wait_timeout)
    rospy.sleep(0.2)

    if args.send_enable:
        rospy.loginfo('publishing enable flag...')
        for _ in range(5):
            enable_pub.publish(Bool(data=True))
            rospy.sleep(0.05)
        rospy.sleep(0.3)

    for cycle_idx in range(args.cycles):
        rospy.loginfo('cycle %d/%d: open both grippers', cycle_idx + 1, args.cycles)
        move_grippers(
            left_pub,
            right_pub,
            left_cache,
            right_cache,
            clamp(args.open_pos, 0.0, 0.08),
            clamp(args.open_pos, 0.0, 0.08),
            args.move_time,
            args.rate,
        )
        rospy.sleep(args.pause_time)

        rospy.loginfo('cycle %d/%d: close both grippers', cycle_idx + 1, args.cycles)
        move_grippers(
            left_pub,
            right_pub,
            left_cache,
            right_cache,
            clamp(args.close_pos, 0.0, 0.08),
            clamp(args.close_pos, 0.0, 0.08),
            args.move_time,
            args.rate,
        )
        rospy.sleep(args.pause_time)

    rospy.loginfo('done')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
