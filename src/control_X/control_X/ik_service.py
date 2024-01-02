from custom_interfaces.srv import IkSolver  


import rclpy
from rclpy.node import Node
import math
import numpy as np
from math import atan2, sin, cos, sqrt , pi
 
#takes in an euler angle (ZYX) and returns the x column of the rotation matrix
def EulerToRot(x, y, z):
        zRot = [[math.cos(z), math.sin(z), 0],
                [-math.sin(z), math.cos(z), 0],
                [0, 0, 1]]
        
        yRot = [[math.cos(y), 0, -math.sin(y)],
                [0, 1, 0],
                [math.sin(y), 0, math.cos(y)]]
        
        xRot = [[1, 0, 0],
                [0, math.cos(x), math.sin(x)],
                [0, -math.sin(x), math.cos(x)]]

        Rot = np.dot(xRot, np.dot(yRot,zRot))
        invRot = np.linalg.inv(Rot)
        #print(invRot)
        xVector = [invRot[0][0], invRot[1][0], invRot[2][0]]
        return(xVector)

#takes in a vector and returns the angle from the vector to the horizontal plane
def phiAngle(xVector):
     phi = np.arctan2(xVector[2],(np.sqrt(xVector[0]**2 + xVector[1]**2)))
     return phi

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(IkSolver, 'topic_inverse', self.ik_callback)       

    
    def ik_callback(self, request, response):                                                
        self.get_logger().info('Incoming request\na: %f b: %f c: %f d: %f e: %f f: %f g: %f' % (request.x, request.y, request.z, request.qx, request.qy, request.qz, request.qw))  

        q = [0, 0, 0, 0, 0, 0, 0]
        q[0]=request.x
        q[1]=request.y
        q[2]=request.z
        q[3]=request.qx
        q[4]=request.qy
        q[5]=request.qz
        q[6]=request.qw

        angles = [0, 0, 0]

        #link lengths
        l1 = 0.05716 #0.096326 old value
        l2 = 0.13023
        l3 = 0.124
        l4 = 0.1334

        #Quaternions to Euler angles (ZYX)
        # roll (x-axis rotation)
        sinr_cosp = 2 * (q[6] * q[3] + q[4] * q[5])
        cosr_cosp = 1 - 2 * (q[3] * q[3] + q[4] * q[4])
        angles[0] = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = math.sqrt(1 + 2 * (q[6] * q[4] - q[3] * q[5]))
        cosp = math.sqrt(1 - 2 * (q[6] * q[4] - q[3] * q[5]))
        angles[1] = 2 * math.atan2(sinp, cosp) - math.pi / 2

        # yaw (z-axis rotation)
        siny_cosp = 2 * (q[6] * q[5] + q[3] * q[4])
        cosy_cosp = 1 - 2 * (q[4] * q[4] + q[5] * q[5])
        angles[2] = math.atan2(siny_cosp, cosy_cosp)      

        #print('roll: ', angles[0], ', pitch: ', angles[1], ', yaw: ', angles[2])


        x = EulerToRot(angles[0], angles[1], angles[2])
        phi = phiAngle(x)
        #print("phi: " + str(phi))
        
        theta1 = np.arctan2(q[1], q[0])
        alpha = np.sqrt(q[0]**2 + q[1]**2)
        #print("alpha: " + str(alpha))

        #wAlpha and wZ are for the wrist joint relative to joint 2
        wAlpha = alpha - l4*math.cos(phi)
        wZ = (q[2] - l1) - l4*math.sin(phi)
        #print("wAlpha: " + str(wAlpha))
        #print("wZ: " + str(wZ))

        preTheta3 = np.arccos((wAlpha**2 + wZ**2 -(l2**2 + l3**2))/(2*l2*l3))
        preTheta2 = np.arctan2(wZ, wAlpha) - np.arctan2((l3*math.sin(preTheta3)), (l2 + l3*math.cos(preTheta3)))

        theta4 = phi - preTheta3 - preTheta2 +1.38

        #adjusted values for home configuration
        theta3 = -preTheta3 + 1.00544838 +0.18
        theta2 = -preTheta2 +0.1

        response.theta1 = theta1
        response.theta2 = theta2
        response.theta3 = theta3
        response.theta4 = theta4

        
        print("values are", response.theta1, response.theta2 , response.theta3 , response.theta4)
        return response
        # #asssume sum, theta2+theta3+theta4=phi
        # phi =0
        # theta_vals= find_ik([xc,yc,zc,phi])

    
        # response.theta1 = theta_vals[0]
        # response.theta2 = theta_vals[1]
        # response.theta3 = theta_vals[2]
        # response.theta4 = theta_vals[3]
        # # for i in range(len(theta_vals)):
        # #     print("joint values are ", theta_vals[i][0] ,theta_vals[i][1] , theta_vals[i][2] , theta_vals[i][3] )
        # print("joint values are ", response.theta1 ,response.theta2 , response.theta3 , response.theta4 )
        
        # return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()