import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
import sys


class BasicRobotControl(Node):
    def __init__(self):
        super().__init__('basic_robot_control')
        self.armclient = self.create_client(SetJointPosition, 'goal_joint_space_path')
        while not self.armclient.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')
        self.send_arm_request()
        
        # self.toolclient = self.create_client(SetJointPosition, 'goal_tool_control')
        # while not self.toolclient.wait_for_service(timeout_sec=1.0):
        #     if not rclpy.ok():
        #         self.get_logger().error('Interrupted while waiting for the service. Exiting.')
        #         sys.exit(0)
        #     self.get_logger().info('Service not available, waiting again...')
        # self.send_tool_request()

    def send_arm_request(self):
        request = SetJointPosition.Request()
        request.planning_group = 'asrm'
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        request.joint_position.position = [0.6, -0.2, 0.0, 0.8, 0.0]
        request.path_time = 5.0

        self.future = self.armclient.call_async(request)
        
    # def send_tool_request(self):
    #     request = SetJointPosition.Request()
    #     request.planning_group = 'asrm'
    #     request.joint_position.joint_name = ['gripper']
    #     request.joint_position.position = [0.003]
    #     request.path_time = 5.0

    #     self.future = self.toolclient.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    basic_robot_control = BasicRobotControl()

    while rclpy.ok():
        rclpy.spin_once(basic_robot_control)
        if basic_robot_control.future.done():
            try:
                response = basic_robot_control.future.result()
            except Exception as e:
                basic_robot_control.get_logger().error('Service call failed %r' % (e,))
            break

    basic_robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()