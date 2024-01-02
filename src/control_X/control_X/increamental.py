import sys

import rclpy
from rclpy.node import Node
from custom_interfaces.srv import JkVelocity , JkJoint 
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetJointPosition
from sensor_msgs.msg import JointState
import time





        
        
class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(JkJoint, 'jk_joint')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('joint service not available, waiting again...')
        self.req = JkJoint.Request()
        
        self.subscription = self.create_subscription(
            JointState, #get info in type Float32MultiArray
            '/joint_states', #listens to topic "topic_inverse"
            self.joint_value_callback, #its callback gets called as soon as it receives a message
            10)
        self.subscription  # prevent unused variable warning
        
        self.armclient = self.create_client(SetJointPosition, 'goal_joint_space_path')
        while not self.armclient.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('set position Service not available, waiting again...')
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.theta3 = 0.0
        self.theta4 = 0.0
        time.sleep(5)
        for i in range(10):
            print("sending_request")
            print("joint values", self.theta1,self.theta2,self.theta3,self.theta4)
            response = self.send_request(1.0,0.0,0.0,0.0,0.0,-0.0 )
            joint_pos = self.increament(response.theta1 ,response.theta2, response.theta3, response.theta4)
            print(joint_pos)
            self.send_arm_request(joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3])
            time.sleep(1)

    def send_request(self, vx, vy, vz, wx, wy, wz):
        self.req.vx = vx
        self.req.vy = vy
        self.req.vz = vz
        self.req.wx = wx
        self.req.wy = wy
        self.req.wz = wz
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def send_arm_request(self, t1, t2, t3, t4):
        request = SetJointPosition.Request()
        request.planning_group = 'asrm'
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        request.joint_position.position = [t1, t2, t3, t4]
        request.path_time = 5.0

        self.future = self.armclient.call_async(request)
        
    def increament(self, v1, v2, v3, v4):
        joint_velocity = [v1, v2, v3, v4]
        q_ref = [self.theta1 ,self.theta2 ,self.theta3 ,self.theta4  ]
        sampling_time = 0.1
        q_new=[]
        while(True):
            q_new.append(q_ref[0] + joint_velocity[0] * sampling_time)
            q_new.append(q_ref[1] + joint_velocity[1] * sampling_time)
            q_new.append(q_ref[2] + joint_velocity[2] * sampling_time)
            
            q_new.append(q_ref[3] + joint_velocity[3] * sampling_time)
            
            return q_new
            
    def joint_value_callback(self, msg):
        q = msg.position

        self.theta1 = q[0]
        self.theta2 = q[1]
        self.theta3 = q[2]
        self.theta4 = q[3]

def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    
    # minimal_client.get_logger().info(
    #     'Result of add_two_ints: for %d + %d = %d' %
    #     (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
