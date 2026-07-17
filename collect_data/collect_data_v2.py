# -*- coding: UTF-8 -*-
"""
多频率数据采集脚本 v2
- 图像采集: ~30Hz (受相机帧率限制)
- 关节采集: ~200Hz (高频FOC反馈)
- 所有数据带时间戳，支持后期对齐
"""
import os
import time
import numpy as np
import h5py
import argparse
import threading
from collections import deque
from typing import Optional, Dict, List, Tuple

import rospy
from sensor_msgs.msg import JointState, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2


class MultiFreqDataCollector:
    """多频率数据采集器"""
    
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()
        
        # 数据缓冲区 (使用锁保护线程安全)
        self.lock = threading.Lock()
        
        # 图像缓冲区 (带时间戳)
        self.img_front_buffer: List[Tuple[float, np.ndarray]] = []
        self.img_left_buffer: List[Tuple[float, np.ndarray]] = []
        self.img_right_buffer: List[Tuple[float, np.ndarray]] = []
        
        # 深度图像缓冲区 (可选)
        self.img_front_depth_buffer: List[Tuple[float, np.ndarray]] = []
        self.img_left_depth_buffer: List[Tuple[float, np.ndarray]] = []
        self.img_right_depth_buffer: List[Tuple[float, np.ndarray]] = []
        
        # 关节数据缓冲区 (带时间戳) - 从臂 puppet
        self.joint_left_buffer: List[Tuple[float, np.ndarray, np.ndarray, np.ndarray]] = []  # (ts, pos, vel, eff)
        self.joint_right_buffer: List[Tuple[float, np.ndarray, np.ndarray, np.ndarray]] = []
        
        # 主臂数据缓冲区 (用于 action) - 主臂 master
        self.master_left_buffer: List[Tuple[float, np.ndarray]] = []  # (ts, pos)
        self.master_right_buffer: List[Tuple[float, np.ndarray]] = []
        
        # 底盘数据缓冲区 (可选)
        self.base_buffer: List[Tuple[float, np.ndarray]] = []  # (ts, [vx, wz])
        
        # 采集控制
        self.is_collecting = False
        self.start_time: Optional[float] = None
        
        # 初始化ROS
        self._init_ros()
        
    def _init_ros(self):
        """初始化ROS节点和订阅"""
        rospy.init_node('multifreq_data_collector', anonymous=True)
        
        # 图像订阅 (30Hz)
        rospy.Subscriber(self.args.img_front_topic, Image, 
                        self._img_front_callback, queue_size=100, tcp_nodelay=True)
        rospy.Subscriber(self.args.img_left_topic, Image, 
                        self._img_left_callback, queue_size=100, tcp_nodelay=True)
        rospy.Subscriber(self.args.img_right_topic, Image, 
                        self._img_right_callback, queue_size=100, tcp_nodelay=True)
        
        # 深度图像订阅 (可选)
        if self.args.use_depth_image:
            rospy.Subscriber(self.args.img_front_depth_topic, Image,
                            self._img_front_depth_callback, queue_size=100, tcp_nodelay=True)
            rospy.Subscriber(self.args.img_left_depth_topic, Image,
                            self._img_left_depth_callback, queue_size=100, tcp_nodelay=True)
            rospy.Subscriber(self.args.img_right_depth_topic, Image,
                            self._img_right_depth_callback, queue_size=100, tcp_nodelay=True)
        
        # 从臂关节订阅 (200Hz) - 包含 qpos, qvel, effort
        rospy.Subscriber(self.args.puppet_arm_left_topic, JointState,
                        self._puppet_left_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.puppet_arm_right_topic, JointState,
                        self._puppet_right_callback, queue_size=1000, tcp_nodelay=True)
        
        # 主臂关节订阅 (200Hz) - 只需要 position 作为 action
        rospy.Subscriber(self.args.master_arm_left_topic, JointState,
                        self._master_left_callback, queue_size=1000, tcp_nodelay=True)
        rospy.Subscriber(self.args.master_arm_right_topic, JointState,
                        self._master_right_callback, queue_size=1000, tcp_nodelay=True)
        
        # 底盘订阅 (可选)
        if self.args.use_robot_base:
            rospy.Subscriber(self.args.robot_base_topic, Odometry,
                            self._base_callback, queue_size=1000, tcp_nodelay=True)
        
        rospy.loginfo("MultiFreq Data Collector initialized")
        
    # ==================== 图像回调 ====================
    def _img_front_callback(self, msg: Image):
        if not self.is_collecting:
            return
        ts = msg.header.stamp.to_sec()
        img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        with self.lock:
            self.img_front_buffer.append((ts, img.copy()))
            
    def _img_left_callback(self, msg: Image):
        if not self.is_collecting:
            return
        ts = msg.header.stamp.to_sec()
        img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        with self.lock:
            self.img_left_buffer.append((ts, img.copy()))
            
    def _img_right_callback(self, msg: Image):
        if not self.is_collecting:
            return
        ts = msg.header.stamp.to_sec()
        img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        with self.lock:
            self.img_right_buffer.append((ts, img.copy()))
            
    def _img_front_depth_callback(self, msg: Image):
        if not self.is_collecting:
            return
        ts = msg.header.stamp.to_sec()
        img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        with self.lock:
            self.img_front_depth_buffer.append((ts, img.copy()))
            
    def _img_left_depth_callback(self, msg: Image):
        if not self.is_collecting:
            return
        ts = msg.header.stamp.to_sec()
        img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        with self.lock:
            self.img_left_depth_buffer.append((ts, img.copy()))
            
    def _img_right_depth_callback(self, msg: Image):
        if not self.is_collecting:
            return
        ts = msg.header.stamp.to_sec()
        img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        with self.lock:
            self.img_right_depth_buffer.append((ts, img.copy()))
    
    # ==================== 关节回调 (从臂 - 高频) ====================
    def _puppet_left_callback(self, msg: JointState):
        if not self.is_collecting:
            return
        ts = msg.header.stamp.to_sec()
        pos = np.array(msg.position, dtype=np.float32)
        vel = np.array(msg.velocity, dtype=np.float32) if len(msg.velocity) > 0 else np.zeros_like(pos)
        eff = np.array(msg.effort, dtype=np.float32) if len(msg.effort) > 0 else np.zeros_like(pos)
        with self.lock:
            self.joint_left_buffer.append((ts, pos, vel, eff))
            
    def _puppet_right_callback(self, msg: JointState):
        if not self.is_collecting:
            return
        ts = msg.header.stamp.to_sec()
        pos = np.array(msg.position, dtype=np.float32)
        vel = np.array(msg.velocity, dtype=np.float32) if len(msg.velocity) > 0 else np.zeros_like(pos)
        eff = np.array(msg.effort, dtype=np.float32) if len(msg.effort) > 0 else np.zeros_like(pos)
        with self.lock:
            self.joint_right_buffer.append((ts, pos, vel, eff))
    
    # ==================== 主臂回调 (用于action) ====================
    def _master_left_callback(self, msg: JointState):
        if not self.is_collecting:
            return
        ts = msg.header.stamp.to_sec()
        pos = np.array(msg.position, dtype=np.float32)
        with self.lock:
            self.master_left_buffer.append((ts, pos))
            
    def _master_right_callback(self, msg: JointState):
        if not self.is_collecting:
            return
        ts = msg.header.stamp.to_sec()
        pos = np.array(msg.position, dtype=np.float32)
        with self.lock:
            self.master_right_buffer.append((ts, pos))
    
    # ==================== 底盘回调 ====================
    def _base_callback(self, msg: Odometry):
        if not self.is_collecting:
            return
        ts = msg.header.stamp.to_sec()
        vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.angular.z], dtype=np.float32)
        with self.lock:
            self.base_buffer.append((ts, vel))
    
    # ==================== 采集控制 ====================
    def clear_buffers(self):
        """清空所有缓冲区"""
        with self.lock:
            self.img_front_buffer.clear()
            self.img_left_buffer.clear()
            self.img_right_buffer.clear()
            self.img_front_depth_buffer.clear()
            self.img_left_depth_buffer.clear()
            self.img_right_depth_buffer.clear()
            self.joint_left_buffer.clear()
            self.joint_right_buffer.clear()
            self.master_left_buffer.clear()
            self.master_right_buffer.clear()
            self.base_buffer.clear()
            
    def start_collecting(self):
        """开始采集"""
        self.clear_buffers()
        self.start_time = rospy.Time.now().to_sec()
        self.is_collecting = True
        rospy.loginfo("Started collecting data...")
        
    def stop_collecting(self):
        """停止采集"""
        self.is_collecting = False
        rospy.loginfo("Stopped collecting data")
        
    def get_status(self) -> Dict[str, int]:
        """获取当前缓冲区状态"""
        with self.lock:
            return {
                'img_front': len(self.img_front_buffer),
                'img_left': len(self.img_left_buffer),
                'img_right': len(self.img_right_buffer),
                'joint_left': len(self.joint_left_buffer),
                'joint_right': len(self.joint_right_buffer),
                'master_left': len(self.master_left_buffer),
                'master_right': len(self.master_right_buffer),
                'base': len(self.base_buffer),
            }
    
    # ==================== 数据处理与保存 ====================
    def _align_joint_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        对齐左右臂关节数据 (基于时间戳)
        返回: (timestamps, qpos, qvel, effort, actions)
        """
        # 获取左右臂数据
        left_data = list(self.joint_left_buffer)
        right_data = list(self.joint_right_buffer)
        master_left = list(self.master_left_buffer)
        master_right = list(self.master_right_buffer)
        
        if len(left_data) == 0 or len(right_data) == 0:
            raise ValueError("No joint data collected!")
        
        # 使用左臂时间戳作为基准，为每个时间点找最近的右臂数据
        left_ts = np.array([d[0] for d in left_data])
        right_ts = np.array([d[0] for d in right_data])
        master_left_ts = np.array([d[0] for d in master_left]) if master_left else left_ts
        master_right_ts = np.array([d[0] for d in master_right]) if master_right else right_ts
        
        aligned_ts = []
        aligned_qpos = []
        aligned_qvel = []
        aligned_effort = []
        aligned_actions = []
        
        for i, t in enumerate(left_ts):
            # 找最近的右臂数据
            right_idx = np.argmin(np.abs(right_ts - t))
            if np.abs(right_ts[right_idx] - t) > 0.02:  # 超过20ms认为不匹配
                continue
                
            # 找最近的主臂数据 (用于action)
            master_left_idx = np.argmin(np.abs(master_left_ts - t)) if len(master_left) > 0 else i
            master_right_idx = np.argmin(np.abs(master_right_ts - t)) if len(master_right) > 0 else right_idx
            
            # 合并左右臂数据
            left_pos, left_vel, left_eff = left_data[i][1], left_data[i][2], left_data[i][3]
            right_pos, right_vel, right_eff = right_data[right_idx][1], right_data[right_idx][2], right_data[right_idx][3]
            
            qpos = np.concatenate([left_pos, right_pos])
            qvel = np.concatenate([left_vel, right_vel])
            effort = np.concatenate([left_eff, right_eff])
            
            # Action: 主臂位置 (如果没有主臂数据，使用从臂位置)
            if len(master_left) > 0 and len(master_right) > 0:
                action = np.concatenate([master_left[master_left_idx][1], master_right[master_right_idx][1]])
            else:
                action = qpos.copy()  # 默认使用从臂位置
            
            aligned_ts.append(t)
            aligned_qpos.append(qpos)
            aligned_qvel.append(qvel)
            aligned_effort.append(effort)
            aligned_actions.append(action)
        
        return (
            np.array(aligned_ts, dtype=np.float64),
            np.array(aligned_qpos, dtype=np.float32),
            np.array(aligned_qvel, dtype=np.float32),
            np.array(aligned_effort, dtype=np.float32),
            np.array(aligned_actions, dtype=np.float32),
        )
    
    def _process_images(self, camera_buffer: List[Tuple[float, np.ndarray]]) -> Tuple[np.ndarray, np.ndarray]:
        """处理图像缓冲区，返回 (timestamps, images)"""
        if len(camera_buffer) == 0:
            return np.array([]), np.array([])
        timestamps = np.array([d[0] for d in camera_buffer], dtype=np.float64)
        images = np.stack([d[1] for d in camera_buffer], axis=0)
        return timestamps, images
    
    def save_episode(self, dataset_path: str):
        """保存一个episode的数据"""
        t0 = time.time()
        
        with self.lock:
            # 处理关节数据
            joint_ts, qpos, qvel, effort, actions = self._align_joint_data()
            
            # 处理图像数据
            img_front_ts, img_front = self._process_images(self.img_front_buffer)
            img_left_ts, img_left = self._process_images(self.img_left_buffer)
            img_right_ts, img_right = self._process_images(self.img_right_buffer)
            
            # 处理深度图像 (可选)
            if self.args.use_depth_image:
                img_front_depth_ts, img_front_depth = self._process_images(self.img_front_depth_buffer)
                img_left_depth_ts, img_left_depth = self._process_images(self.img_left_depth_buffer)
                img_right_depth_ts, img_right_depth = self._process_images(self.img_right_depth_buffer)
            
            # 处理底盘数据 (可选)
            if self.args.use_robot_base and len(self.base_buffer) > 0:
                base_ts = np.array([d[0] for d in self.base_buffer], dtype=np.float64)
                base_vel = np.stack([d[1] for d in self.base_buffer], axis=0)
            else:
                base_ts = np.array([])
                base_vel = np.array([])
        
        # 计算实际采样频率
        if len(joint_ts) > 1:
            joint_freq = len(joint_ts) / (joint_ts[-1] - joint_ts[0])
        else:
            joint_freq = 0
            
        if len(img_front_ts) > 1:
            image_freq = len(img_front_ts) / (img_front_ts[-1] - img_front_ts[0])
        else:
            image_freq = 0
        
        duration = joint_ts[-1] - joint_ts[0] if len(joint_ts) > 1 else 0
        
        # 保存到HDF5
        with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
            # 元数据
            root.attrs['sim'] = False
            root.attrs['compress'] = False
            root.attrs['multifreq'] = True  # 标记为多频率数据
            root.attrs['image_freq'] = image_freq
            root.attrs['joint_freq'] = joint_freq
            root.attrs['duration'] = duration
            
            # ========== 图像组 ==========
            images_grp = root.create_group('images')
            
            # cam_high (front)
            cam_high = images_grp.create_group('cam_high')
            if len(img_front) > 0:
                cam_high.create_dataset('data', data=img_front, dtype='uint8',
                                       chunks=(1, img_front.shape[1], img_front.shape[2], 3))
                cam_high.create_dataset('timestamps', data=img_front_ts, dtype='float64')
            
            # cam_left_wrist
            cam_left = images_grp.create_group('cam_left_wrist')
            if len(img_left) > 0:
                cam_left.create_dataset('data', data=img_left, dtype='uint8',
                                       chunks=(1, img_left.shape[1], img_left.shape[2], 3))
                cam_left.create_dataset('timestamps', data=img_left_ts, dtype='float64')
            
            # cam_right_wrist
            cam_right = images_grp.create_group('cam_right_wrist')
            if len(img_right) > 0:
                cam_right.create_dataset('data', data=img_right, dtype='uint8',
                                        chunks=(1, img_right.shape[1], img_right.shape[2], 3))
                cam_right.create_dataset('timestamps', data=img_right_ts, dtype='float64')
            
            # 深度图像 (可选)
            if self.args.use_depth_image:
                images_depth_grp = root.create_group('images_depth')
                
                cam_high_d = images_depth_grp.create_group('cam_high')
                if len(img_front_depth) > 0:
                    cam_high_d.create_dataset('data', data=img_front_depth, dtype='uint16')
                    cam_high_d.create_dataset('timestamps', data=img_front_depth_ts, dtype='float64')
                
                cam_left_d = images_depth_grp.create_group('cam_left_wrist')
                if len(img_left_depth) > 0:
                    cam_left_d.create_dataset('data', data=img_left_depth, dtype='uint16')
                    cam_left_d.create_dataset('timestamps', data=img_left_depth_ts, dtype='float64')
                
                cam_right_d = images_depth_grp.create_group('cam_right_wrist')
                if len(img_right_depth) > 0:
                    cam_right_d.create_dataset('data', data=img_right_depth, dtype='uint16')
                    cam_right_d.create_dataset('timestamps', data=img_right_depth_ts, dtype='float64')
            
            # ========== 关节组 ==========
            joints_grp = root.create_group('joints')
            joints_grp.create_dataset('timestamps', data=joint_ts, dtype='float64')
            joints_grp.create_dataset('qpos', data=qpos, dtype='float32')
            joints_grp.create_dataset('qvel', data=qvel, dtype='float32')
            joints_grp.create_dataset('effort', data=effort, dtype='float32')
            
            # ========== 动作组 ==========
            actions_grp = root.create_group('actions')
            actions_grp.create_dataset('timestamps', data=joint_ts, dtype='float64')  # 与关节同频
            actions_grp.create_dataset('data', data=actions, dtype='float32')
            
            # ========== 底盘组 (可选) ==========
            if len(base_ts) > 0:
                base_grp = root.create_group('base')
                base_grp.create_dataset('timestamps', data=base_ts, dtype='float64')
                base_grp.create_dataset('velocity', data=base_vel, dtype='float32')
        
        print(f'\033[32m\nSaved: {time.time() - t0:.1f}s')
        print(f'  Path: {dataset_path}.hdf5')
        print(f'  Duration: {duration:.2f}s')
        print(f'  Images: {len(img_front_ts)} frames ({image_freq:.1f} Hz)')
        print(f'  Joints: {len(joint_ts)} frames ({joint_freq:.1f} Hz)')
        print(f'  Actions: {len(actions)} frames\033[0m\n')


def get_arguments():
    parser = argparse.ArgumentParser(description='Multi-frequency data collection for robot learning')
    
    # 数据路径
    parser.add_argument('--dataset_dir', type=str, default='./data',
                       help='Directory to save dataset')
    parser.add_argument('--task_name', type=str, default='multifreq_task',
                       help='Task name')
    parser.add_argument('--episode_idx', type=int, default=0,
                       help='Episode index')
    
    # 采集时长
    parser.add_argument('--duration', type=float, default=30.0,
                       help='Maximum collection duration in seconds')
    
    # 相机话题
    parser.add_argument('--img_front_topic', type=str, default='/camera_f/color/image_raw')
    parser.add_argument('--img_left_topic', type=str, default='/camera_l/color/image_raw')
    parser.add_argument('--img_right_topic', type=str, default='/camera_r/color/image_raw')
    
    # 深度相机话题
    parser.add_argument('--img_front_depth_topic', type=str, default='/camera_f/depth/image_raw')
    parser.add_argument('--img_left_depth_topic', type=str, default='/camera_l/depth/image_raw')
    parser.add_argument('--img_right_depth_topic', type=str, default='/camera_r/depth/image_raw')
    
    # 从臂关节话题 (puppet - 包含 qpos, qvel)
    parser.add_argument('--puppet_arm_left_topic', type=str, default='/puppet/joint_left')
    parser.add_argument('--puppet_arm_right_topic', type=str, default='/puppet/joint_right')
    
    # 主臂关节话题 (master - 用于action)
    parser.add_argument('--master_arm_left_topic', type=str, default='/master/joint_left')
    parser.add_argument('--master_arm_right_topic', type=str, default='/master/joint_right')
    
    # 底盘话题
    parser.add_argument('--robot_base_topic', type=str, default='/odom')
    
    # 功能开关
    parser.add_argument('--use_robot_base', action='store_true', default=False)
    parser.add_argument('--use_depth_image', action='store_true', default=False)
    
    return parser.parse_args()


def main():
    args = get_arguments()
    
    # 创建数据目录
    dataset_dir = os.path.join(args.dataset_dir, args.task_name)
    if not os.path.exists(dataset_dir):
        os.makedirs(dataset_dir)
    
    # 初始化采集器
    collector = MultiFreqDataCollector(args)
    
    # 等待数据流
    print("\n等待数据流就绪...")
    rospy.sleep(2.0)
    
    # 交互式采集
    print("\n" + "="*60)
    print("多频率数据采集器 v2")
    print("="*60)
    print("命令:")
    print("  s - 开始/停止采集")
    print("  p - 打印缓冲区状态")
    print("  d - 丢弃当前数据")
    print("  w - 保存当前数据")
    print("  q - 退出")
    print("="*60 + "\n")
    
    while not rospy.is_shutdown():
        try:
            cmd = input("输入命令: ").strip().lower()
            
            if cmd == 's':
                if not collector.is_collecting:
                    collector.start_collecting()
                else:
                    collector.stop_collecting()
                    status = collector.get_status()
                    print(f"采集完成: {status}")
                    
            elif cmd == 'p':
                status = collector.get_status()
                print(f"缓冲区状态: {status}")
                
            elif cmd == 'd':
                collector.clear_buffers()
                print("已清空缓冲区")
                
            elif cmd == 'w':
                if collector.is_collecting:
                    collector.stop_collecting()
                
                status = collector.get_status()
                if status['joint_left'] == 0:
                    print("\033[31m错误: 没有数据可保存\033[0m")
                    continue
                
                dataset_path = os.path.join(dataset_dir, f"episode_{args.episode_idx}")
                collector.save_episode(dataset_path)
                args.episode_idx += 1
                collector.clear_buffers()
                
            elif cmd == 'q':
                print("退出...")
                break
                
            else:
                print(f"未知命令: {cmd}")
                
        except KeyboardInterrupt:
            print("\n中断")
            break
        except EOFError:
            break


if __name__ == '__main__':
    main()
