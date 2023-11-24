import rclpy
from rclpy.node import Node
from scripts.pointperfect_module import PointPerfectModule
from scripts.serial_module import UbloxSerial
from sensor_msgs.msg import NavSatFix
import threading
from queue import Queue

class TallysmanGPSPublisher(Node):
    def __init__(self, args = None):
        super().__init__('tallysman_gps_publisher')

        self.declare_parameter('usb_port','/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 38400)
        self.declare_parameter('topic_name', 'gps_data')

        usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        topic_name = self.get_parameter("topic_name").get_parameter_value().string_value

        self.publisher_ = self.create_publisher(NavSatFix, topic_name, 50)
        self.ser = UbloxSerial(usb_port, baud_rate, self.get_logger())
        self.ser.nmea_message_found += self.read_nmea_message
        self.ser.rtcm_message_found += self.read_rtcm_message
        self.lock = threading.Lock()
        self.data_queue = Queue()

        self.pp = PointPerfectModule(self.get_logger(), self.correction_message_received)
        # Starting a separate pointperfect thread for pointperfect process
        self.pp_thread = threading.Thread(target=self.pp.process)
        self.pp_thread.daemon = True
        self.pp_thread.start()

        # Create a timer that calls the 'publish_gps_data' method every 0.1 seconds (10 Hz).
        self.timer = self.create_timer(1, self.ser.poll_messages)

    def correction_message_received(self, client, userdata, message):
        self.get_logger().info(message="[PointPerfect] : " + message.payload.hex(' '))
        self.ser.send(message.payload)
        pass

    def read_nmea_message(self, nmeaMessage):
        #self.data_queue.put(nmeaMessage)
        latitude = nmeaMessage.lat if nmeaMessage.lat else None
        longitude = nmeaMessage.lon if nmeaMessage.lon else None
        if latitude is not None and longitude is not None:
            msg = NavSatFix()
            msg.latitude = float(latitude)
            msg.longitude = float(longitude)
            self.publisher_.publish(msg)
            self.get_logger().info('Published GPS data - Latitude: {:.6f}, Longitude: {:.6f}'.format(latitude, longitude))           

    def read_rtcm_message(self, rtcmMessage):
        pass

def main(args=None):
    rclpy.init(args=args)
    tallysman_gps_publisher = TallysmanGPSPublisher(args=args)
    rclpy.spin(tallysman_gps_publisher)
    tallysman_gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
