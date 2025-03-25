import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from wlkata_mirobot import WlkataMirobot,WlkataMirobotTool
class MirobotTeleopNode(Node):
    def __init__(self):

        
        super().__init__('mirobot_teleop_node')
        
        # 初始化0 Mirobot 机械臂
        self.arm = WlkataMirobot(portname='/dev/ttyUSB0')
        # self.arm.set_soft_limit(True)
        # #self.arm.home()  # 机械臂归零
        
        # self.arm.set_tool_type(WlkataMirobotTool.GRIPPER)
        # self.arm.set_gripper(38400)
        # self.arm.gripper_close()  # 初始化时关闭夹爪
        # #self.gripper_opened = False  # 夹爪状态记录
        
        # 订阅 cmd_vel 话题
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 订阅 /joy 话题
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # 初始化按钮状态
        self.last_button7_state = 0  # 第七个按钮的初始状态
        self.last_button8_state = 0  # 第八个按钮message_print的初始状态
        self.last_button3_state = 0  # 第三个按钮的初始状态
        self.last_button5_state = 0  # 第五个按钮的初始状态
        
        # 初始化缩放因子
        self.linear_scale = 10  # 默认线性缩放因子
        self.angular_scale = 5  # 默认角度缩放因子
        self.alternate_linear_scale = 30  # 备用线性缩放因子
        self.alternate_angular_scale = 10  # 备用角度缩放因子
    def cmd_vel_callback(self, msg):
        """
        处理 cmd_vel 话题的回调函数，控制机械臂的移动。
        """
        # 获取 linear 和 angular 数据
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        linear_z = msg.linear.z
        
        angular_x = msg.angular.x
        angular_y = msg.angular.y
        angular_z = msg.angular.z
        
        # 将 linear 数据转换为机械臂的 XYZ 移动
        target_x = linear_x * self.linear_scale  # 使用当前缩放因子
        target_y = linear_y * self.linear_scale
        target_z = linear_z * self.linear_scale
        
        # 将 angular 数据转换为机械臂的姿态控制
        target_roll = angular_x * self.angular_scale  # 使用当前缩放因子
        target_pitch = angular_y * self.angular_scale
        target_yaw = angular_z * self.angular_scale
        print(target_x,target_y,target_z,target_roll,target_pitch,target_yaw)
        # 设置机械臂的末端位姿
        self.arm.set_tool_pose(
            x=target_x, y=target_y, z=target_z,
            roll=target_roll, pitch=target_pitch, yaw=target_yaw,
            is_relative=True,wait_ok=False
        )
        
    def joy_callback(self, msg):
        """
        处理 /joy 话题的回调函数，控制夹爪的开合和缩放因子的切换。
        """
        # 获取按钮状态
        buttons = msg.buttons  # buttons 是一个数组，格式为 array('i', [0, 0, 0, ...])
        
        # 检查第七个按钮是否被按下（索引为 6）
        if len(buttons) > 6:
            current_button7_state = buttons[6]  # 第七个按钮的状态
            if current_button7_state != self.last_button7_state:
                if current_button7_state == 1:
                    self.arm.gripper_open()  # 第七个按钮按下，夹爪打开
                    print("gopen")
                self.last_button7_state = current_button7_state  # 更新按钮状态
        
        # 检查第八个按钮是否被按下（索引为 7）
        if len(buttons) > 7:
            current_button8_state = buttons[7]  # 第八个按钮的状态
            if current_button8_state != self.last_button8_state:
                if current_button8_state == 1:
                    self.arm.gripper_close()  # 第八个按钮按下，夹爪关闭
                    print("gopen")
                self.last_button8_state = current_button8_state  # 更新按钮状态
        
        # 检查第2个按钮是否被按下（索引为 1）
        if len(buttons) > 1:
            current_button3_state = buttons[1]  # 第三个按钮的状态
            if current_button3_state != self.last_button3_state:
                if current_button3_state == 1:
                    # 切换到备用缩放因子
                    self.linear_scale = self.alternate_linear_scale
                    self.angular_scale = self.alternate_angular_scale
                    self.get_logger().info("切换到备用缩放因子")
                    print("切换到备用缩放因子")
                self.last_button3_state = current_button3_state  # 更新按钮状态
        
        # 检查第五个按钮是否被按下（索引为 4）
        if len(buttons) > 4:
            current_button5_state = buttons[4]  # 第五个按钮的状态
            if current_button5_state != self.last_button5_state:
                if current_button5_state == 1:
                    # 切换回默认缩放因子
                    self.linear_scale = 10
                    self.angular_scale = 10
                    self.get_logger().info("切换回默认缩放因子")
                    print("切换回默认缩放因子")
                self.last_button5_state = current_button5_state  # 更新按钮状态

def main(args=None):
    rclpy.init(args=args)
    node = MirobotTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()