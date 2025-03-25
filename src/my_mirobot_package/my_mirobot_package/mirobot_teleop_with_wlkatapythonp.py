import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from sensor_msgs.msg import Joy # type: ignore
from std_msgs.msg import String  # Import the String message type # type: ignore
from .Mirobot_UART import Mirobot_UART  # Import the Mirobot_UART class # type: ignore
import serial
from mirobot_work_define.msg import MirobotStatus, MirobotAction # type: ignore
from mirobot_work_define.srv import StartRecording # type: ignore
from std_srvs.srv import Empty # type: ignore
import time  # Import the time module

class MirobotTeleopNode(Node):
    def __init__(self):
        super().__init__('mirobot_teleop_node')

        # Declare parameters
        self.declare_parameter('mirobot_tools', 2)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')

        # Get parameters
        self.mirobot_tools = self.get_parameter('mirobot_tools').get_parameter_value().integer_value
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value

        # Initialize Mirobot using Mirobot_UART
        self.arm = Mirobot_UART()
        self.arm.init(serial.Serial(self.serial_port, 115200), -1)  # Use parameter for serial port
        print(self.arm.message_print(True))
        print(self.arm.version())
        # self.arm.sendMsg("$h")
        self.arm.sendMsg("$20=1")
        self.arm.sendMsg("$21=1")
        self.arm.speed(2000)
        if self.mirobot_tools == 1:
            self.gripper_opened = False
            self.arm.gripper(2)  # Close gripper initially
        if self.mirobot_tools == 2:
            self.pump_status = 0
            self.arm.pump(0)  # Close gripper initially
        # 定义QoS配置
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,  # 设置缓冲区大小为10
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # 允许消息丢失
            durability=rclpy.qos.DurabilityPolicy.VOLATILE  # 不持久化消息
        )


        # Subscribe to cmd_vel topic
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_o',
            self.cmd_vel_callback,
            qos_profile=qos_profile
        )
        
        # Subscribe to /joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Create a publisher for the robot status using MirobotStatus
        self.status_pub = self.create_publisher(MirobotStatus, '/mirobot_status', 10)

        # Create a publisher for the /mirobot_action topic
        self.action_pub = self.create_publisher(MirobotAction, '/mirobot_action', 10)

        # Initialize button states
        self.last_button7_state = 0
        self.last_button8_state = 0
        self.last_button1_state = 0
        self.last_button2_state = 0
        self.last_button3_state = 0
        self.last_button5_state = 0
        self.last_button12_state = 0 
        self.last_button11_state = 0
        # Initialize scaling factors
        self.linear_scale = 5
        self.angular_scale = 3
        self.alternate_linear_scale = 10
        self.alternate_angular_scale = 5
        self.last_h_command_time = 0  # Initialize the last command time

        self.last_button3_time = 0

        # Declare a parameter for the instructions file path
        self.declare_parameter('instructions_file', 'instructions.txt')

        # Get the instructions file path parameter
        instructions_file = self.get_parameter('instructions_file').get_parameter_value().string_value

        # Load instructions from the specified file
        self.instructions = self.load_instructions(instructions_file)
        
        self.current_instruction_index = 0

        # Initialize the parameter client
        self.param_client = self.create_client(Empty, '/data_recorder')
        self.set_language_instruction(self.instructions[0])

        # Initialize recording state
        self.is_recording = False

        # Load episode name from file
        self.episode_name = self.load_episode_name('episode_name.txt')

    def cmd_vel_callback(self, msg):
        """
        Callback function for cmd_vel topic, controls the robot arm's movement.
        """
        # Get linear and angular data
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        linear_z = msg.linear.z
        
        angular_x = msg.angular.x
        angular_y = msg.angular.y
        angular_z = msg.angular.z
        
        # Convert linear data to robot arm's XYZ movement
        target_x = linear_x * self.linear_scale  # Use current scaling factor
        target_y = linear_y * self.linear_scale
        target_z = linear_z * self.linear_scale
        
        # Convert angular data to robot arm's pose control
        target_roll = angular_x * self.angular_scale
        target_pitch = angular_y * self.angular_scale
        target_yaw = angular_z * self.angular_scale
        #print(target_x, target_y, target_z, target_roll, target_pitch, target_yaw)
        
        # Set the robot arm's end effector pose
        self.arm.writecoordinate(
            motion=0,  # Linear movement
            position=1,  # Relative movement
            x=target_x, y=target_y, z=target_z,
            a=target_roll, b=target_pitch, c=target_yaw
        )
        # Publish the action
        tool_type = self.mirobot_tools  # Assuming self.mirobot_tools is defined elsewhere
        tool_status = self.pump_status if self.mirobot_tools == 2 else self.gripper_opened
        self.publish_action(target_x, target_y, target_z, target_roll, target_pitch, target_yaw, tool_type, tool_status)
        
        # Publish the status
        status = self.arm.getStatus()
        #print(status)
        if status == -1:
            pass
        else:
            status_msg = self.parse_status(status)
            self.status_pub.publish(status_msg)
            #print(status_msg)

    def parse_status(self, status):
        # Parse the status dictionary into a MirobotStatus message
        status_msg = MirobotStatus()
        status_msg.state = str(status['state'])
        status_msg.angle_a = float(status['angle_A'])
        status_msg.angle_b = float(status['angle_B'])
        status_msg.angle_c = float(status['angle_C'])
        status_msg.angle_d = float(status['angle_D'])
        status_msg.angle_x = float(status['angle_X'])
        status_msg.angle_y = float(status['angle_Y'])
        status_msg.angle_z = float(status['angle_Z'])
        status_msg.coordinate_x = float(status['coordinate_X'])
        status_msg.coordinate_y = float(status['coordinate_Y'])
        status_msg.coordinate_z = float(status['coordinate_Z'])
        status_msg.coordinate_rx = float(status['coordinate_RX'])
        status_msg.coordinate_ry = float(status['coordinate_RY'])
        status_msg.coordinate_rz = float(status['coordinate_RZ'])
        status_msg.pump = bool(int(status['pump']))
        status_msg.valve = bool(int(status['valve']))
        status_msg.mooe = int(status['mooe'])
        return status_msg
    def publish_action(self, target_x, target_y, target_z, target_roll, target_pitch, target_yaw, tool_type, tool_status):
        action_msg = MirobotAction()
        action_msg.target_x = target_x
        action_msg.target_y = target_y
        action_msg.target_z = target_z
        action_msg.target_roll = target_roll
        action_msg.target_pitch = target_pitch
        action_msg.target_yaw = target_yaw
        action_msg.tool_type = tool_type
        action_msg.tool_status = tool_status
        self.action_pub.publish(action_msg)
    def joy_callback(self, msg):
        """
        Callback function for /joy topic, controls gripper and scaling factors.
        """
        # Get button states
        buttons = msg.buttons  # buttons is an array, e.g., [0, 0, 0, ...]
        if self.mirobot_tools == 1 or self.mirobot_tools == 2:
            # 检查按钮7的状态（索引6）
            if len(buttons) > 6:
                current_button7_state = buttons[6]  # 按钮7状态
                if current_button7_state != self.last_button7_state:
                    if current_button7_state == 1:
                        if self.mirobot_tools == 1:
                            self.arm.gripper(1)
                            self.gripper_opened = True
                            print("Gripper opened")
                        elif self.mirobot_tools == 2:
                            self.arm.pump(1)
                            self.pump_status = 1
                            print("Pump suction")
                    if current_button7_state == 0:
                        if self.mirobot_tools == 2:
                            self.arm.pump(0)
                            self.pump_status = 0
                            print("Pump off")
                    self.last_button7_state = current_button7_state
            
            if len(buttons) > 7:
                current_button8_state = buttons[7]
                if current_button8_state != self.last_button8_state:
                    if current_button8_state == 1:
                        if self.mirobot_tools == 1:
                            self.arm.gripper(2)
                            self.gripper_opened = False
                            print("Gripper closed")
                        elif self.mirobot_tools == 2:
                            self.arm.pump(2)
                            self.pump_status = 2
                            print("Pump blowing")
                    if current_button8_state == 0:
                        if self.mirobot_tools == 2:
                            self.arm.pump(0)
                            self.pump_status = 0
                            print("Pump off")
                    self.last_button8_state = current_button8_state
        if len(buttons) > 0:
            current_button1_state = buttons[0]
            if current_button1_state != self.last_button1_state:
                if current_button1_state == 0:
                    self.arm.sendMsg("M21 G90 G01 X0 Y0 Z0 A0 B0 C0 F2000")
                    self.get_logger().info("Zero the robot")
                self.last_button1_state = current_button1_state
        
        if len(buttons) > 1:
            current_button2_state = buttons[1]
            if current_button2_state != self.last_button2_state:
                if current_button2_state == 0:
                    self.linear_scale = self.alternate_linear_scale
                    self.angular_scale = self.alternate_angular_scale
                    self.get_logger().info("Switched to alternate scaling factors")
                    #print("Switched to alternate scaling factors")
                self.last_button2_state = current_button2_state
        
        if len(buttons) > 4:
            current_button5_state = buttons[4]
            if current_button5_state != self.last_button5_state:
                if current_button5_state == 0:
                    self.linear_scale = 10
                    self.angular_scale = 5
                    self.get_logger().info("Switched back to default scaling factors")
                    #print("Switched back to default scaling factors")
                self.last_button5_state = current_button5_state

        # 开始/结束录制
        if len(buttons) > 10:
            current_button11_state = buttons[10]
            if current_button11_state != self.last_button11_state:
                if current_button11_state == 1:
                    if not self.is_recording:
                        # Start recording
                        client = self.create_client(StartRecording, '/start_recording')
                        request = StartRecording.Request()
                        request.episode_name = self.episode_name
                        client.call_async(request)
                        self.is_recording = True
                        self.get_logger().info(f"Started recording: {self.episode_name}")
                    else:
                        # Stop recording
                        client = self.create_client(Empty, '/stop_recording')
                        client.call_async(Empty.Request())
                        self.is_recording = False
                        self.get_logger().info("Stopped recording")
                self.last_button11_state = current_button11_state

        # # 结束录制并回家    DON'T USE THIS!!! it will be make wrong
        # if len(buttons) > 10:
        #     current_button11_state = buttons[10]
        #     if current_button11_state != self.last_button11_state:
        #         if current_button11_state == 1:
        #             current_time = time.time()
        #             if current_time - self.last_h_command_time > 5:
        #                 client = self.create_client(Empty, '/stop_recording')
        #                 client.call_async(Empty.Request())
        #                 self.arm.sendMsg("$h")
        #                 self.arm.sendMsg("G01 F2000")
        #                 self.last_h_command_time = current_time
        #         self.last_button11_state = current_button11_state

        # Check for button press to reset instruction index and reload instructions and episode_name
        if len(buttons) > 11:  # Assuming button 12 is used for resetting instructions
            current_button12_state = buttons[11]
            if current_button12_state == 1 and current_button12_state != self.last_button12_state:
                # Reload instructions from the file
                instructions_file = self.get_parameter('instructions_file').get_parameter_value().string_value
                self.instructions = self.load_instructions(instructions_file)
                self.set_language_instruction(self.instructions[0])
                # Reset current_instruction_index
                self.current_instruction_index = 0
                # Reset episode_name
                self.episode_name = self.load_episode_name('episode_name.txt')
                self.get_logger().info(f"episode_name, Instructions reloaded and current_instruction_index reset to 0")
                #print(f"Instructions reloaded and current_instruction_index reset to 0")
            self.last_button12_state = current_button12_state
        if len(buttons) > 3:
            current_button3_state = buttons[3]
            if current_button3_state == 1 and current_button3_state != self.last_button3_state:
                # Ensure at least 1.5 seconds between button presses
                if time.time() - self.last_button3_time > 1.5:
                    # Increment the instruction index to move forward
                    self.current_instruction_index = (self.current_instruction_index + 1) % len(self.instructions)
                    new_instruction = self.instructions[self.current_instruction_index]
                    self.set_language_instruction(new_instruction)
                    self.last_button3_time = time.time()
            self.last_button3_state = current_button3_state

    def load_instructions(self, file_path):
        with open(file_path, 'r') as file:
            return [line.strip() for line in file.readlines()]

    def set_language_instruction(self, instruction):
        # Set the new instruction as a parameter
        self.param_client.call_async(Empty.Request())
        self.get_logger().info(f"Set language_instruction to: {instruction}")

    def load_episode_name(self, file_path):
        with open(file_path, 'r') as file:
            episode_name = file.readline().strip()
            self.get_logger().info(f"episode_name: {episode_name}")
            return episode_name

def main(args=None):
    rclpy.init(args=args)
    node = MirobotTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()