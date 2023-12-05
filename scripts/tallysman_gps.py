import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ByteMultiArray
from collections import UserList
import threading
from queue import Queue
from scripts.pointperfect_module import PointPerfectModule
from scripts.serial_module import UbloxSerial
from pynmeagps import NMEAMessage
from pyrtcm import RTCMMessage, RTCMReader

class TallysmanGPSPublisher(Node):
    def __init__(self, args = None) -> None:
        super().__init__('tallysman_gps_publisher')

        #region Parameters declaration
        self.declare_parameter('usb_port','/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 38400)
        self.declare_parameter('topic_name', 'gps_data')
        self.declare_parameter('is_base', True) # Parameter {use_corrections} needs to be present if this is True
        self.declare_parameter('use_corrections', True) # Parameter {config_path} needs to be present if this is True.
        self.declare_parameter('region', 'us') # The region where antenna is present. Parameter only needed when use_corrections is True
        self.declare_parameter('config_path', '') # The path where the corrections_config is placed. Parameter only needed when use_corrections is True
        self.declare_parameter('rtcm_topic_name', 'rtcm_corrections') # This should be unique for a base/rover pair.
        #endregion

        #region Parameters Initialization
        usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        region = self.get_parameter("region").get_parameter_value().string_value
        self.use_corrections = self.get_parameter("use_corrections").get_parameter_value().bool_value
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self.is_base = self.get_parameter("is_base").get_parameter_value().bool_value
        rtcm_topic_name = self.get_parameter("rtcm_topic_name").get_parameter_value().string_value
        #endregion

        self.ser = UbloxSerial(usb_port, baud_rate, self.get_logger(), self.is_base, self.use_corrections)

        #region Conditional attachments to events based on rover/base
        if self.is_base:
            self.ser.rtcm_message_found += self.handle_rtcm_message
            
            # Establishing Pointperfect connection only if it's enabled. Required parameters needs to be sent.
            if self.use_corrections:
                self.pp = PointPerfectModule(self.get_logger(), config_path, region)
                self.pp.on_correction_message += self.handle_correction_message       
            
            # Publisher to publish RTCM corrections to Rover
            self.rtcm_publisher = self.create_publisher(ByteMultiArray, rtcm_topic_name, 50)     
        else:
            self.ser.nmea_message_found += self.handle_nmea_message

            # Subscriber for receiving RTCM corrections from base.
            self.rtcm_subscriber = self.create_subscription(ByteMultiArray, rtcm_topic_name, self.handle_rtcm_message, 50)
            
            # Publisher for location information. Taking location from rover since it is more accurate.
            self.publisher = self.create_publisher(NavSatFix, topic_name, 50)
        #endregion

        self.lock = threading.Lock()

        # Timer to poll status messages from base/rover for every sec.
        self.timer = self.create_timer(1, self.ser.poll_messages)
        pass

    """
        Handles the correction messages received from PointPerfect MQTT connection.

        The decoding of correction messages and applying them to the location is done by the antenna itself.
        Need to make sure we send entire message to the antenna without a delay, Since the correction messages are time dependent.
    """
    def handle_correction_message(self, message) -> None:
        self.get_logger().info(message="[PointPerfect] : " + message.hex(' '))
        self.ser.send(message)
        pass
    
    """
        Handles the received NMEAMessage.

        Location information is taken from RMC and GGA messages and published to the {topic_name} topic via NavSatFix message
    """
    def handle_nmea_message(self, nmeaMessage: NMEAMessage) -> None:
        if nmeaMessage.identity == "GNRMC" or nmeaMessage.identity == "GNGGA":
            latitude = nmeaMessage.lat if nmeaMessage.lat else None
            longitude = nmeaMessage.lon if nmeaMessage.lon else None
            if latitude is not None and longitude is not None:
                msg = NavSatFix()
                msg.latitude = float(latitude)
                msg.longitude = float(longitude)
                self.publisher.publish(msg)
                self.get_logger().info('Published GPS data - Latitude: {:.6f}, Longitude: {:.6f}'.format(latitude, longitude))           
        pass
    
    """
        Handles received RTCM message. 
        
        If the antenna is base, The received RTCM message is from antenna and is published to the {rtcm_topic_name} topic via ByteMultiArray message.
        If the antenna is rover, The received RTCM message is from {rtcm_topic_name} topic via ByteMultiArray message and is sent to the antenna through serial port.
    """
    def handle_rtcm_message(self, rtcmMessage) -> None:
        if self.is_base:
            msg = ByteMultiArray()
            convertedList = []
            for val in list(rtcmMessage.serialize()):
                convertedList.append(val.to_bytes(1, 'big'))
            msg.data = UserList(convertedList)
            self.rtcm_publisher.publish(msg)
            self.get_logger().info('Published RTCM message with identity: ' + rtcmMessage.identity) 
        else:
            data = b''.join(rtcmMessage.data)
            self.ser.send(data)
            rmg = RTCMReader.parse(data)
            self.get_logger().info('Received RTCM message: ' + rmg.identity)
        pass

    def poll_messages(self) -> None:
        self.ser.poll_messages(self.is_base)
        pass

def main(args=None):
    rclpy.init(args=args)
    tallysman_gps_publisher = TallysmanGPSPublisher(args=args)
    rclpy.spin(tallysman_gps_publisher)
    tallysman_gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
