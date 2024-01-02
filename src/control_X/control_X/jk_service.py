# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from sensor_msgs.msg import JointState

import math
import rclpy
from rclpy.node import Node
import numpy as np #import numpy for matrix calculations

from std_msgs.msg import Float32MultiArray #import Float32MultiArray to allow the use of arrays
from sensor_msgs.msg import JointState
from custom_interfaces.srv import JkVelocity , JkJoint 


#finds the Z column for each frame
#theta1, theta2... are the joint values
#n is which frame its on, ex. n=3 means frame 3
def DHparameterZ(theta1, theta2, theta3, theta4, n):
        angles = [theta1, theta2-1.5708, theta3+1.5708, theta4] #angles array initiated to be the first 3 numbers the user inputs, corresponding to the joint angles
        d = [0.05716, 0, 0, 0]
        a = [0, 0.128, 0.148, 0.1334]
        alpha = [-1.5708, 0, 0, 0]
        
        TransMatrix = [[1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]
                ]
        i = 0
        while i < n:
            TransMatrix = np.dot(TransMatrix, [[math.cos(angles[i]), -math.sin(angles[i])*math.cos(alpha[i]), math.sin(angles[i])*math.sin(alpha[i]), a[i]*math.cos(angles[i])],
                                [math.sin(angles[i]), math.cos(angles[i])*math.cos(alpha[i]), -math.cos(angles[i])*math.sin(alpha[i]), a[i]*math.sin(angles[i])],
                                [0, math.sin(alpha[i]), math.cos(alpha[i]), d[i]],
                                [0, 0, 0, 1]
                ])
            i = i + 1

        # print(new) #first multiplies A2 by A3, then multiplies A1 with the result of A2*A3. Then prints the matrix
    
        #x, y, z values for end effector pose orientation
        dx = TransMatrix[0][3]
        dy = TransMatrix[1][3]
        dz = TransMatrix[2][3]  
        #print("Forward Kinematics topic")
        #print("end effector pose orientation values are:\n x:"+ str(dx) + ",\n y:" + str(dy) + ",\n z:" + str(dz))

        #quaternian values for end effector pose orientation 
        q_w= 0.5 * math.sqrt(1 + TransMatrix[0][0] + TransMatrix[1][1] + TransMatrix[2][2])
        q_x= (TransMatrix[2][1] - TransMatrix[1][2]) / (4 * q_w) 
        q_y= (TransMatrix[0][2] - TransMatrix[2][0]) / (4 * q_w)
        q_z= (TransMatrix[1][0] -TransMatrix[0][2]) / (4 * q_w)     

        #print("end effector pose orientation values are: \n x: "+ str(q_x) + ",\n y: " + str(q_y) + ",\n z: " + str(q_z) + ",\n w:" + str(q_w)+"\n")
        
        zn = [TransMatrix[0][2], TransMatrix[1][2], TransMatrix[2][2]]
        return zn

#finds the O position for each frame
#theta1, theta2... are the joint values
#n is which frame its on, ex. n=3 means frame 3
def DHparameterO(theta1, theta2, theta3, theta4, n):
        angles = [theta1, theta2-1.5708, theta3+1.5708, theta4] #angles array initiated to be the first 3 numbers the user inputs, corresponding to the joint angles
        d = [0.05716, 0, 0, 0]
        a = [0, 0.128, 0.148, 0.1334]
        alpha = [-1.5708, 0, 0, 0]
        
        TransMatrix = [[1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]
                ]
        i = 0
        while i < n:
            TransMatrix = np.dot(TransMatrix, [[math.cos(angles[i]), -math.sin(angles[i])*math.cos(alpha[i]), math.sin(angles[i])*math.sin(alpha[i]), a[i]*math.cos(angles[i])],
                                [math.sin(angles[i]), math.cos(angles[i])*math.cos(alpha[i]), -math.cos(angles[i])*math.sin(alpha[i]), a[i]*math.sin(angles[i])],
                                [0, math.sin(alpha[i]), math.cos(alpha[i]), d[i]],
                                [0, 0, 0, 1]
                ])
            i = i + 1

        # print(new) #first multiplies A2 by A3, then multiplies A1 with the result of A2*A3. Then prints the matrix
    
        #x, y, z values for end effector pose orientation
        dx = TransMatrix[0][3]
        dy = TransMatrix[1][3]
        dz = TransMatrix[2][3]  
        #print("Forward Kinematics topic")
        #print("end effector pose orientation values are:\n x:"+ str(dx) + ",\n y:" + str(dy) + ",\n z:" + str(dz))

        #quaternian values for end effector pose orientation 
        q_w= 0.5 * math.sqrt(1 + TransMatrix[0][0] + TransMatrix[1][1] + TransMatrix[2][2])
        q_x= (TransMatrix[2][1] - TransMatrix[1][2]) / (4 * q_w) 
        q_y= (TransMatrix[0][2] - TransMatrix[2][0]) / (4 * q_w)
        q_z= (TransMatrix[1][0] -TransMatrix[0][2]) / (4 * q_w)     

        #print("end effector pose orientation values are: \n x: "+ str(q_x) + ",\n y: " + str(q_y) + ",\n z: " + str(q_z) + ",\n w:" + str(q_w)+"\n")
        
        on = [dx, dy, dz]
        return on

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber') #super().__init__ calls the Node class’s constructor and gives it the name minimal_publisher
        
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.theta3 = 0.0
        self.theta4 = 0.0
        self.srv1 = self.create_service(JkJoint, 'jk_joint', self.jk_joint_callback)
        
        self.srv2 = self.create_service(JkVelocity, 'jk_velocity', self.jk_vel_callback)
        self.subscription = self.create_subscription(
            JointState, #get info in type Float32MultiArray
            '/joint_states', #listens to topic "topic_inverse"
            self.joint_value_callback, #its callback gets called as soon as it receives a message
            10)
        self.subscription  # prevent unused variable warning

    def jk_vel_callback(self, request, response):                                                
        self.get_logger().info('Incoming request\na: %f b: %f c: %f d: %f ' % (request.joint_v1, request.joint_v2, request.joint_v3, request.joint_v4))  

        
        joint_v1 = request.joint_v1 
        joint_v2 = request.joint_v2 
        joint_v3 = request.joint_v3 
        joint_v4 = request.joint_v4 
        
        
        theta1 = self.theta1
        theta2 = self.theta2
        theta3 = self.theta3
        theta4 = self.theta4
        
        o1 = DHparameterO(theta1, theta2, theta3, theta4, 1)
        o2 = DHparameterO(theta1, theta2, theta3, theta4, 2)
        o3 = DHparameterO(theta1, theta2, theta3, theta4, 3)
        o4 = DHparameterO(theta1, theta2, theta3, theta4, 4)

        z0 = [1, 0, 0]
        z1 = DHparameterZ(theta1, theta2, theta3, theta4, 1)
        z2 = DHparameterZ(theta1, theta2, theta3, theta4, 2)
        z3 = DHparameterZ(theta1, theta2, theta3, theta4, 3)
        z4 = DHparameterZ(theta1, theta2, theta3, theta4, 4)

        Jv1 = np.cross(z0, o4)
        Jv2 = np.cross(z1, np.subtract(o4, o1))
        Jv3 = np.cross(z2, np.subtract(o4, o2))
        Jv4 = np.cross(z3, np.subtract(o4, o3))

        Jw1 = z0
        Jw2 = z1
        Jw3 = z2
        Jw4 = z3

        Jacobian1 = np.concatenate((Jv1, Jw1), axis=None)
        Jacobian2 = np.concatenate((Jv2, Jw2), axis=None)
        Jacobian3 = np.concatenate((Jv3, Jw3), axis=None)
        Jacobian4 = np.concatenate((Jv4, Jw4), axis=None)
        # print(Jacobian1)

        #this combines all the jacobian columns together, ignore how messy this is
        Jacobian = np.vstack((Jacobian1, Jacobian2, Jacobian3, Jacobian4))
        Jacobian = Jacobian.T
        # Jacobian = np.transpose(np.concatenate((np.concatenate((np.concatenate((Jacobian1, Jacobian2), axis=1), Jacobian3), axis=1), Jacobian4), axis=1))
        # print("jacobian",Jacobian.shape)
        # print("")
        joint_values = np.array([joint_v1, joint_v2, joint_v3, joint_v4])
        joint_values = joint_values.T
        # print("joint ", joint_values.shape)
        velocity_matrix = np.dot(Jacobian,joint_values)
        
        response.vx  =velocity_matrix[0]
        response.vy  =velocity_matrix[1]
        response.vz  =velocity_matrix[2]
        response.wx  =velocity_matrix[3]
        response.wy  =velocity_matrix[4]
        response.wz  =velocity_matrix[5]

        print("End effector linear Velocity values are", response.vx, response.vy , response.vz )
        print("End effector angular Velocity values are", response.wx, response.wy , response.wz )
        
        return response
    
    def jk_joint_callback(self, request, response):                                                
        self.get_logger().info('Incoming request\na: %f b: %f c: %f d: %f e: %f f: %f ' % (request.vx, request.vy, request.vz, request.wx, request.wy, request.wz))  

        vx = request.vx
        vy = request.vy
        vz = request.vz
        wx = request.wx
        wy = request.wy
        wz = request.wz
        
        theta1 = self.theta1
        theta2 = self.theta2
        theta3 = self.theta3
        theta4 = self.theta4
        
        o1 = DHparameterO(theta1, theta2, theta3, theta4, 1)
        o2 = DHparameterO(theta1, theta2, theta3, theta4, 2)
        o3 = DHparameterO(theta1, theta2, theta3, theta4, 3)
        o4 = DHparameterO(theta1, theta2, theta3, theta4, 4)

        z0 = [1, 0, 0]
        z1 = DHparameterZ(theta1, theta2, theta3, theta4, 1)
        z2 = DHparameterZ(theta1, theta2, theta3, theta4, 2)
        z3 = DHparameterZ(theta1, theta2, theta3, theta4, 3)
        z4 = DHparameterZ(theta1, theta2, theta3, theta4, 4)

        Jv1 = np.cross(z0, o4)
        Jv2 = np.cross(z1, np.subtract(o4, o1))
        Jv3 = np.cross(z2, np.subtract(o4, o2))
        Jv4 = np.cross(z3, np.subtract(o4, o3))

        Jw1 = z0
        Jw2 = z1
        Jw3 = z2
        Jw4 = z3

        Jacobian1 = np.concatenate((Jv1, Jw1), axis=None)
        Jacobian2 = np.concatenate((Jv2, Jw2), axis=None)
        Jacobian3 = np.concatenate((Jv3, Jw3), axis=None)
        Jacobian4 = np.concatenate((Jv4, Jw4), axis=None)
        # print(Jacobian1)

        #this combines all the jacobian columns together, ignore how messy this is
        Jacobian = np.vstack((Jacobian1, Jacobian2, Jacobian3, Jacobian4))
        Jacobian = Jacobian.T
        
        J_inverse = np.linalg.pinv(Jacobian)
        # Jacobian = np.transpose(np.concatenate((np.concatenate((np.concatenate((Jacobian1, Jacobian2), axis=1), Jacobian3), axis=1), Jacobian4), axis=1))
        # print("jacobian",Jacobian.shape)
        
        tip_velocity = np.array([vx, vy, vz, wx, wy, wz])
        joint_velocity = np.dot(J_inverse, tip_velocity)
        # print("")
        joint_velocity = joint_velocity.T
        
        
        response.theta1 = joint_velocity[0]
        response.theta2 = joint_velocity[1]
        response.theta3 = joint_velocity[2]
        response.theta4 = joint_velocity[3]


        print("Joint values are", response.theta1, response.theta2 , response.theta3, response.theta4 )
        
        return response
    
    
    def joint_value_callback(self, msg):
        q = msg.position

        self.theta1 = q[0]
        self.theta2 = q[1]
        self.theta3 = q[2]
        self.theta4 = q[3]
        

        


def main(args=None):
    rclpy.init(args=args) #rclpy library is initialized

    minimal_subscriber = MinimalSubscriber() #the node is created

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
