import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from open_manipulator_msgs.msg import KinematicsPose
import matplotlib.pyplot as plt
import numpy as np


class MinimalSubscriber(Node):

    def __init__(self):
        
        self.dh_values=[]
        self.robot_values = []
        self.dh_x=[]
        self.rb_x=[]
        self.dh_y=[]
        self.rb_y=[]
        self.dh_z=[]
        self.rb_z=[]
        
        super().__init__('minimal_subscriber')
        self.dh_subscription = self.create_subscription(
            Float32MultiArray,
            'dh_topic',
            self.dh_callback,
            10)
        self.dh_subscription  # prevent unused variable warning
        
        self.robot_subscription = self.create_subscription(
            KinematicsPose,
            '/kinematics_pose',
            self.robot_callback,
            10)
        self.robot_subscription  # prevent unused variable warning
        

    def dh_callback(self, msg):
        self.dh_values = msg.data
        print(self.dh_values)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.append)
      
        
    def robot_callback(self, msg):
        position =  msg.pose.position
        self.robot_values = [position.x , position.y , position.z]
        # print(self.robot_values)
        
        
    def append(self):
        
        self.dh_x.append(self.dh_values[0])
        self.rb_x.append(self.robot_values[0])
        self.dh_y.append(self.dh_values[1])
        self.rb_y.append(self.robot_values[1])
        self.dh_z.append(self.dh_values[2])
        self.rb_z.append(self.robot_values[2])
        if len(self.dh_x)==50000:
            position_real_x = np.array(self.dh_x) 
            position_calculated_x = np.array(self.rb_x)
            
            position_real_y = np.array(self.dh_y) 
            position_calculated_y = np.array(self.rb_y)
            
            position_real_z = np.array(self.dh_z) 
            position_calculated_z = np.array(self.rb_z)
            
            
            position_error_x = position_real_x - position_calculated_x
            position_error_y = position_real_y - position_calculated_y
            position_error_z = position_real_z - position_calculated_z
            # Create box and whisker plots
            plt.figure(figsize=(10, 5))

            # Positional error in x boxplot
            plt.subplot(1, 3, 1)
            plt.boxplot(position_error_x, labels=['Positional Error in x'])
            plt.title('Positional Error in x Boxplot')
            plt.ylabel('Error in x')

            # Positional error in y boxplot
            plt.subplot(1, 3, 2)
            plt.boxplot(position_error_y, labels=['Positional Error in y'])
            plt.title('Positional Error in y Boxplot')
            plt.ylabel('Error in y')
            
            # Positional error in z boxplot
            plt.subplot(1, 3, 3)
            plt.boxplot(position_error_z, labels=['Positional Error in z'])
            plt.title('Positional Error in z Boxplot')
            plt.ylabel('Error in z')

            plt.tight_layout()
            plt.show()
        
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()