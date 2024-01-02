import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
import sys
from std_msgs.msg import String
import time


class BasicRobotControl(Node):
    def __init__(self):
        super().__init__('basic_robot_control')
        
        
        self.home_location = [0.0, -0.2, 0.0, 0.8, 0.0]
  
        
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')
        
        self.toolclient = self.create_client(SetJointPosition, 'goal_tool_control')
        while not self.toolclient.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')
        # self.subscription = self.create_subscription(
        #     String,
        #     'goal_box',
        #     self.chess_callback,
        #     10)
        # self.subscription  # prevent unused variable warning
        msg= "0 to 1"
        self.chess_callback(msg)


    def chess_callback(self, msg):
        command = msg
        command = command.split(" to ")
        self.pick_location = int(command[0])
        self.place_location = int(command[1])
        print(msg)
        
        locations= {"pick_inter": [0.6, -0.2, 0.0, 0.8, 0.0], \
                    "pick_location": [0.6, -0.2, 0.72, 0.8, 0.0], \
                    "place_inter": [-0.6, -0.2, 0.0, 0.8, 0.0],\
                    "place_location": [-0.6, -0.2, 0.72, 0.8, 0.0]}
    
        
        pick = [0.000]
        place = [0.009]

        self.execute(locations , pick , place)
        
    def execute(self, locations, pick, place):
        print("going to home location")
        self.send_request(self.home_location)
        time.sleep(5)
        print("gripper open ")
        self.send_tool_request(place)
        time.sleep(5)
        print("going to pick intermediate location")
        self.send_request(locations["pick_inter"])
        time.sleep(5)
        print("going to pick location")
        self.send_request(locations["pick_location"])
        time.sleep(5)
        print("gripper close")
        self.send_tool_request(pick)    
        time.sleep(2)
        print("going to home location")
        self.send_request(self.home_location)
        time.sleep(5)
        print("going to place intermediate location")
        self.send_request(locations["place_inter"])
        time.sleep(5)
        print("going to place location")
        self.send_request(locations["place_location"])
        time.sleep(5)
        print("gripper open")
        self.send_tool_request(place)
        time.sleep(2)
        print("going to pick intermediate location")
        self.send_request(locations["place_inter"])
        time.sleep(5)
        print("going to home location")
        self.send_request(self.home_location)
        time.sleep(5)
        

    def send_request(self, position):
        # print("sending request ", position)
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        request.joint_position.position = position
        request.path_time = 5.0

        self.future = self.client.call_async(request)
        
    def send_tool_request(self, position):
        request = SetJointPosition.Request()
        request.planning_group = 'asrm'
        request.joint_position.joint_name = ['gripper']
        request.joint_position.position = position
        request.path_time = 5.0

        self.future = self.toolclient.call_async(request)
    
    

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
