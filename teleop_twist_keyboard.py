#!/usr/bin/env python3
"""ROS 2 无人机键盘控制：速度经 Bridge 转换，解锁和模式切换直接调用服务。"""

from functools import partial
import os
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint


HELP = """
无人机键盘控制（固定世界 ENU 坐标，不随机头或镜头转向）
  W/S：东/西       A/D：北/南
  Q/E：升/降       Z/C：正/负偏航
  T：解锁         O：OFFBOARD       X：上锁
  +/-：调整速度   空格：停止       Ctrl+C：退出并发送零速度
按住方向键持续运动，停止重复输入后自动减速至零。
"""

MOVE_BINDINGS = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 1, 0, 0),
    'd': (0, -1, 0, 0),
    'q': (0, 0, 1, 0),
    'e': (0, 0, -1, 0),
    'z': (0, 0, 0, 1),
    'c': (0, 0, 0, -1),
}


class KeyboardControl(Node):
    """单节点完成按键解释、速度发布和 MAVROS 服务调用。"""

    def __init__(self):
        """初始化控制参数和 ROS 接口。"""
        super().__init__('teleop_twist_keyboard')
        self.speed = 0.5             # m/s
        self.turn = 1.0              # rad/s
        self.rate = 20.0             # Hz
        self.key_timeout = 0.6       # s
        self.acceleration = 1.0      # m/s²
        self.yaw_acceleration = 2.0  # rad/s²

        self.target = [0.0] * 4
        self.velocity = [0.0] * 4
        self.last_motion = time.monotonic()
        self.state = State()
        self.state.mode = '未知'
        self.result = '等待指令'
        self.publisher = self.create_publisher(MultiDOFJointTrajectoryPoint, 'control/manual', 10)
        self.create_subscription(State, 'mavros/state', self.on_state, 10)
        self.arm_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.command_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.rate, self.publish_command, clock=self.command_clock)

    def on_state(self, message):
        """保存飞机实际状态，供终端显示。

        :param message: MAVROS State 消息。
        """
        self.state = message

    def handle_key(self, key):
        """将按键解释为目标速度或一次服务请求。

        :param key: 终端读到的一个字符，大小写均可。
        """
        key = key.lower()
        if key in MOVE_BINDINGS:
            direction = MOVE_BINDINGS[key]
            self.target = [value * self.speed for value in direction[:3]]
            self.target.append(direction[3] * self.turn)
            self.last_motion = time.monotonic()
        elif key == ' ':
            self.target = [0.0] * 4
        elif key in ('+', '=', '-'):
            factor = 0.9 if key == '-' else 1.1
            self.speed *= factor
            self.turn *= factor
        elif key == 't':
            self.arm(True)
        elif key == 'x':
            self.target = [0.0] * 4
            self.arm(False)
        elif key == 'o':
            self.offboard()

    def publish_command(self):
        """固定频率发布：松键超时后目标归零，再按加减速限制跟随。"""
        if time.monotonic() - self.last_motion > self.key_timeout:
            self.target = [0.0] * 4
        steps = [self.acceleration / self.rate] * 3 + [self.yaw_acceleration / self.rate]
        for index, step in enumerate(steps):
            difference = self.target[index] - self.velocity[index]
            self.velocity[index] += max(-step, min(step, difference))

        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = self.velocity[:3]
        twist.angular.z = self.velocity[3]
        message = MultiDOFJointTrajectoryPoint()
        message.velocities = [twist]
        self.publisher.publish(message)
        status = (f'{self.get_namespace()}  模式={self.state.mode}  解锁={self.state.armed}  '
                  f'速度={self.speed:.2f}  转速={self.turn:.2f}  '
                  f'vx={self.velocity[0]:+.2f} vy={self.velocity[1]:+.2f} '
                  f'vz={self.velocity[2]:+.2f} yaw={self.velocity[3]:+.2f}  {self.result}')
        print('\r\033[2K' + status, end='', flush=True)

    def arm(self, value):
        """异步发送解锁或上锁请求，不阻塞速度发布。

        :param value: True 解锁，False 上锁。
        """
        if not self.arm_client.service_is_ready():
            self.result = '解锁服务尚未就绪'
            return
        request = CommandBool.Request()
        request.value = value
        future = self.arm_client.call_async(request)
        future.add_done_callback(partial(self.on_arm_response, value))

    def on_arm_response(self, value, future):
        """显示服务执行结果，实际解锁状态仍以 State 话题为准。

        :param value: 请求的解锁状态。
        :param future: 服务响应。
        """
        response = future.result()
        action = '解锁' if value else '上锁'
        self.result = action + ('请求成功' if response.success else '请求被拒绝')

    def offboard(self):
        """异步请求 OFFBOARD，保持发布循环持续执行。"""
        if not self.mode_client.service_is_ready():
            self.result = '模式服务尚未就绪'
            return
        request = SetMode.Request()
        request.custom_mode = 'OFFBOARD'
        future = self.mode_client.call_async(request)
        future.add_done_callback(self.on_mode_response)

    def on_mode_response(self, future):
        """显示模式请求结果，终端中的实际模式来自 State。

        :param future: SetMode 服务响应。
        """
        response = future.result()
        self.result = 'OFFBOARD 请求已发送' if response.mode_sent else 'OFFBOARD 请求未发送'

    def stop(self):
        """正常退出前发送一条零速度，保持当前解锁状态不变。"""
        self.target = [0.0] * 4
        self.velocity = [0.0] * 4
        self.publish_command()


def main(args=None):
    """在当前终端运行键盘节点，并在退出时恢复终端设置。

    :param args: ROS 命令行参数。
    :return: 非交互终端返回 1，正常退出返回 0。
    """
    if not sys.stdin.isatty():
        print('键盘控制需要交互终端，请使用 docker compose run --rm keyboard。', file=sys.stderr)
        return 1
    rclpy.init(args=args)
    node = KeyboardControl()
    settings = termios.tcgetattr(sys.stdin)
    try:
        print(HELP, flush=True)
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if select.select([sys.stdin], [], [], 0.01)[0]:
                key = os.read(sys.stdin.fileno(), 1).decode()
                if key == '\x03':
                    break
                node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
