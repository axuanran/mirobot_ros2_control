import os
import argparse
import numpy as np
import cv2
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from rclpy.qos import qos_profile_sensor_data # type: ignore
from sensor_msgs.msg import Image # type: ignore
from cv_bridge import CvBridge # type: ignore
from std_msgs.msg import String # type: ignore
from mirobot_work_define.msg import MirobotStatus, MirobotAction     # type: ignore
from std_srvs.srv import Empty # type: ignore
from mirobot_work_define.srv import StartRecording # type: ignore
import pickle  # Add this import at the top of your file

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')
        self.declare_parameter('data_dir', 'data/train')
        self.data_dir = self.get_parameter('data_dir').get_parameter_value().string_value
        
        # Log the data directory
        self.get_logger().info(f'Data will be saved in: {self.data_dir}')
        
        self.img = None
        self.state = {}
        self.action = {}
        self.video_dir = os.path.join(self.data_dir, 'video')
        os.makedirs(self.video_dir, exist_ok=True)
        self.data = []
        self.is_recording = False
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)
        self.status_sub = self.create_subscription(MirobotStatus, '/mirobot_status', self.status_callback, qos_profile_sensor_data)
        self.action_sub = self.create_subscription(MirobotAction, '/mirobot_action', self.action_callback, qos_profile_sensor_data)
        self.lang_instr_param = self.declare_parameter('language_instruction', 'dummy instruction')
        self.video_writer = None
        self.img_count = 0
        self.create_service(StartRecording, 'start_recording', self.start_recording_callback)  # Use custom service
        self.create_service(Empty, 'stop_recording', self.stop_recording_callback)
        self.timer = self.create_timer(0.2, self.record_callback)  # 5Hz

    def start_recording_callback(self, req, res):
        # Reset the cache and other relevant variables
        self.data = []
        self.img = None
        self.state = {}
        self.action = {}
        
        self.episode_name = req.episode_name  # Get episode_name from the request
        self.is_recording = True
        self.video_writer = cv2.VideoWriter(os.path.join(self.video_dir, f'episode_{self.episode_name}.mp4'),
                                            cv2.VideoWriter_fourcc(*'mp4v'), 10, (640, 480))  # Adjust resolution as needed
        self.get_logger().info(f'Recording started for episode {self.episode_name}')
        return res

    def stop_recording_callback(self, req, res):
        self.is_recording = False
        if self.video_writer:
            self.video_writer.release()
        
        # Save the data using pickle
        with open(os.path.join(self.data_dir, f'episode_{self.episode_name}.pkl'), 'wb') as f:
            pickle.dump(self.data, f)
        
        self.get_logger().info(f'Recording stopped and data saved for episode {self.episode_name}')
        return res

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.video_writer:
            self.video_writer.write(cv_image)
        resized_image = cv2.resize(cv_image, (224, 224))
        self.img = resized_image

    def status_callback(self, msg):
        self.state = {
            'state': msg.state,
            'angle_a': msg.angle_a, #4
            'angle_b': msg.angle_b, #5
            'angle_c': msg.angle_c, #6
            'angle_d': msg.angle_d, #7*
            'angle_x': msg.angle_x, #1
            'angle_y': msg.angle_y, #2
            'angle_z': msg.angle_z, #3
            'coordinate_x': msg.coordinate_x,
            'coordinate_y': msg.coordinate_y,
            'coordinate_z': msg.coordinate_z,
            'coordinate_rx': msg.coordinate_rx,
            'coordinate_ry': msg.coordinate_ry,
            'coordinate_rz': msg.coordinate_rz,
            'pump': msg.pump,
            'valve': msg.valve,
            'mooe': msg.mooe
        }

    def action_callback(self, msg):
        self.action = {
            'target_x': msg.target_x,
            'target_y': msg.target_y,
            'target_z': msg.target_z,
            'target_roll': msg.target_roll,
            'target_pitch': msg.target_pitch,
            'target_yaw': msg.target_yaw,
            'tool_type': msg.tool_type,
            'tool_status': msg.tool_status
        }

    def record_callback(self):
        if self.is_recording:
            if self.img is None:
                self.get_logger().error('Image not initialized.')
                return
            if not self.state:
                self.get_logger().error('State not initialized.')
                return
            if not self.action:
                self.get_logger().error('Action not initialized.')
                return

            try:
                lang_instr = self.get_parameter('language_instruction').value
            except:
                lang_instr = 'dummy instruction'
            
            step_data = {
                'image': self.img,  # image is a 256*256*3 image
                'state': self.state,  # Save the entire state dictionary
                'action': self.action,  # Save the entire action dictionary
                'language_instruction': lang_instr
            }
            self.data.append(step_data)
            self.get_logger().info(f'Recorded step {len(self.data)}')

def main(args=None):
    rclpy.init(args=args)
    recorder = DataRecorder()
    rclpy.spin(recorder)
    recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()