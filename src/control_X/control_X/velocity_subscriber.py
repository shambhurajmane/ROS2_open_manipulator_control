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

import math
import rclpy
from rclpy.node import Node
import numpy as np #import numpy for matrix calculations

from std_msgs.msg import Float32MultiArray #import Float32MultiArray to allow the use of arrays
from sensor_msgs.msg import JointState

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
        self.subscription = self.create_subscription(
            Float32MultiArray, #get info in type Float32MultiArray
            'velocity_topic', #listens to topic "topic_inverse"
            self.listener_callback, #its callback gets called as soon as it receives a message
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        q = msg.data

        theta1 = q[0]
        theta2 = q[1]
        theta3 = q[2]
        theta4 = q[3]
        velocity1 = q[4]
        velocity2 = q[5]
        velocity3 = q[6]
        velocity4 = q[7]

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

        #this combines all the jacobian columns together, ignore how messy this is
        Jacobian = np.transpose(np.concatenate((np.concatenate((np.concatenate((Jacobian1, Jacobian2), axis=0), Jacobian3), axis=0), Jacobian4), axis=0))

        print(Jacobian)


def main(args=None):
    rclpy.init(args=args) #rclpy library is initialized

    minimal_subscriber = MinimalSubscriber() #the node is created

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
