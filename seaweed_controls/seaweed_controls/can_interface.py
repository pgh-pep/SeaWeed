import can
import rclpy
from rclpy.node import Node
from seaweed_interfaces.msg import Command

class CANInterface(Node): 
    def __init__(self, interface='socketcan', channel='can0' , bitrate='500000'):
        super().__init__('can_interface')
        self.subscription = self.create_subscription(
            Command,
            'topic',
            self.process_command,
            10)
        self.subscription
        self.bus = can.Bus(interface=interface, channel=channel, bitrate=bitrate)


    def process_command(self, cmd):
        
        msg = can.Message(arbitration_id=id, data=data)
        try:
            self.bus.send(msg)
        except can.CanError:
            print("Message NOT sent")

    def thrust_command(id, data):
    
    def stop_command(id, data): 

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