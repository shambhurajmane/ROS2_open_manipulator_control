from custom_interfaces.srv import PDSolver  
from dynamixel_sdk_custom_interfaces.msg import SetCurrent
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')

        self.publisher_ = self.create_publisher(SetCurrent, '/set_current', 10) # To publish calculated control_signal
                                                                                # to the /set_current topic
        self.publisher_1 = self.create_publisher(Float32, '/position_data', 10) # This is for plotjuggler to plot the current position
        self.publisher_2 = self.create_publisher(Float32, '/position_ref', 10)  # This is for plotjuggler to plot the reference position
        
        # A client is created to get the current position from robot. 
        self.cli = self.create_client(GetPosition, '/get_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('joint service not available, waiting again...')
        self.req_position = GetPosition.Request()
        
        # A subscriber is created to access the position reference published by first node
        self.subscription = self.create_subscription(
            Float32,
            '/ref', 
            self.position_callback, 
            10)
        self.subscription  # prevent unused variable warning
        self.position =1500  # Initial home position is given to robot, once this node is activated
        self.pd_control()
    
       
    def position_callback(self, msg):          # to access the position reference published by first node                                        
        self.position = msg.data                # the position refernce is stored in self.postion
        print("self.position",self.position)
        
    
    def get_position(self):                     #to get the current position from robot.
        self.req_position.id =14                #Set the id of robot as 14 
        self.future = self.cli.call_async(self.req_position)    #Sent a request to the /get_position service
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def pd_control(self):
        # Define PD controller gains
        msg = SetCurrent()                # msg type for /set_current topic
        pos_data = Float32()               # msg type for /position_data topic
        pos_ref = Float32()                 # msg type for /position_ref topic
        msg.id =14
        Kp = 0.05
        Kd = 0.01

        current_value = self.get_position()
        while abs(self.position - current_value.position)>0.001:
            previous_error = self.position - current_value.position
            current_value = self.get_position()

            # Time step (change in time)
            dt = 0.01

            # Calculate control signal
            print("position is",current_value.position)
            error = self.position - current_value.position

            # Calculate derivative term
            derivative = (error - previous_error) / dt if dt > 0 else 0

            # Calculate control signal
            control_signal = Kp * error + Kd * derivative

            # Update previous error
            previous_error = error
            print(" for iteration", control_signal )

            msg.current =int(control_signal)
            pos_data.data = current_value.position * 1.0
            pos_ref.data = self.position * 1.0
            self.publisher_.publish(msg)            # msg published to /set_current topic
            self.publisher_1.publish(pos_data)       # msg published to /position_data topic
            self.publisher_2.publish(pos_ref)       # msg published to /position_ref topic
            # time.sleep(5)


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()