

import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray

from sensor_msgs.msg import JointState


class QuatSubscriber(Node):

    def __init__(self):
        super().__init__('joint_value_subscriber')
        self.subscription1 = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_value_callback,
            10)
        self.subscription1  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(Float32MultiArray, 'dh_topic', 10)
    



    def joint_value_callback(self, msg):

        
        q = msg.position
        offset_a = (79.38 * math.pi) / 180.0
        
        calib_d = 0 # 0.02633  # from errors calculated for z values, average value= (0.03,0.0277,0.0213)/3 = 0.02633 is subtracted from z  
        calib_a = 0 #0.00673  # from errors calculated for x values, average value= (0.0056,0.014,0.0066)/3 = 0.00673 is subtracted from x  
        angles = [0, q[0], q[1] - offset_a, q[2] + offset_a, q[3]] #angles array initiated to be the first 3 numbers the user inputs, corresponding to the joint angles
        d = [0.036076 -calib_d, 0.06025, 0, 0, 0]       
        a = [0, 0, 0.13023, 0.124, 0.1334 + calib_a]                
        alpha = [0, -math.pi/2, 0, 0, 0]
        
        

        #Homogeneous matrix:
        TransMatrix = [[1, 0, 0, 0],
           [0, 1, 0, 0],
           [0, 0, 1, 0],
           [0, 0, 0, 1]]
        
        for n in [0,1,2,3, 4]:
            TransMatrix = np.dot(TransMatrix, [[math.cos(angles[n]), -math.sin(angles[n])*math.cos(alpha[n]), math.sin(angles[n])*math.sin(alpha[n]), a[n]*math.cos(angles[n])],
                                [math.sin(angles[n]), math.cos(angles[n])*math.cos(alpha[n]), -math.cos(angles[n])*math.sin(alpha[n]), a[n]*math.sin(angles[n])],
                                [0, math.sin(alpha[n]), math.cos(alpha[n]), d[n]],
                                [0, 0, 0, 1]
                ])
        
        #x, y, z values for end effector pose orientation
        dx = TransMatrix[0][3]
        dy = TransMatrix[1][3]
        dz = TransMatrix[2][3]  
        print("Forward Kinematics topic")
        print("end effector pose orientation values are x:"+ str(dx) + ", y:" + str(dy) + ", z:" + str(dz))

        #quaternian values for end effector pose orientation 
        q_w= 0.5 * math.sqrt(1 + TransMatrix[0][0] + TransMatrix[1][1] + TransMatrix[2][2])
        q_x= (TransMatrix[2][1] - TransMatrix[1][2]) / (4 * q_w) 
        q_y= (TransMatrix[0][2] - TransMatrix[2][0]) / (4 * q_w)
        q_z= (TransMatrix[1][0] -TransMatrix[0][2]) / (4 * q_w)   

        print("end effector pose sds are x: "+ str(q_x) + ", y: " + str(q_y) + ", z: " + str(q_z) + ", w:" + str(q_w)+"\n")
        print(TransMatrix)
        msg = Float32MultiArray()
        msg.data = [dx , dy, dz]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    quat_subscriber = QuatSubscriber()

    rclpy.spin(quat_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    quat_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()