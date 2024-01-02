from custom_interfaces.srv import PDSolver  
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('reference_service')
        # Creted a service to accept the reference position value from the user
        self.srv = self.create_service(PDSolver, 'pd_control', self.pd_callback) 
        
        # publishing the reference value on the topic so that teh value will be accessible to second node.
        self.publisher_ = self.create_publisher(Float32, '/ref', 10)
     
    def pd_callback(self, request, response):                                                
        self.get_logger().info('Incoming request\n theta_ref: %f' % (request.theta_ref))  
        msg= Float32()                      # Data type of position reference 
        msg.data = request.theta_ref        # Requetsed value is stored in msg 
        self.publisher_.publish(msg)        # msg is publsihed by publisher
        response.control_ip = 2.0           # Randome value is given as input. If you get time add success
                                            #    variable here and output a String "Success" as a response
        return response
        
def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()