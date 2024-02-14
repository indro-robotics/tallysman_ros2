import time
from typing import Literal
import rclpy
import sys
import base64
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import threading
from tallysman_ros2.pointperfect_module import PointPerfectModule
from tallysman_ros2.serial_module import UbloxSerial
from pynmeagps import NMEAMessage
from pyrtcm import RTCMReader
from tallysman_msg.msg import GnssSignalStatus, RtcmMessage
from tallysman_ros2.logging import Logger, LoggingLevel, SimplifiedLogger

class TallysmanGps(Node):
    def __init__(self, mode:Literal['Disabled', 'Heading_Base', 'Rover']='Disabled') -> None:
        super().__init__('tallysman_gps')
        
        internal_logger = Logger(self.get_logger())
        #region Parameters declaration
        self.declare_parameter('usb_port','/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 230400)
        self.declare_parameter('use_corrections', True) # Parameter {config_path} needs to be present if this is True.
        self.declare_parameter('region', 'us') # The region where antenna is present. Parameter only needed when use_corrections is True
        self.declare_parameter('config_path', '/root/humble_ws/src/tallysman_ros2/pointperfect_files/ucenter-config.json') # The path where the corrections_config is placed. Parameter only needed when use_corrections is True
        self.declare_parameter('save_logs', False)
        self.declare_parameter('log_level', LoggingLevel.Info)
        #endregion

        #region Parameters Initialization
        self.usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.use_corrections = self.get_parameter("use_corrections").get_parameter_value().bool_value
        self.region = self.get_parameter("region").get_parameter_value().string_value
        self.config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self.save_logs = self.get_parameter("save_logs").get_parameter_value().bool_value
        self.log_level : LoggingLevel = LoggingLevel(self.get_parameter("log_level").get_parameter_value().integer_value)
        self.mode : Literal['Disabled', 'Heading_Base', 'Rover'] = mode
        #endregion

        self.process_thread = threading.Thread(target=self.__process, name='tallysman_gps_process', daemon=True)
        self.process_thread.start()

        internal_logger.toggle_logs(self.save_logs)
        internal_logger.setLevel(self.log_level)
        self.logger = SimplifiedLogger(self.mode+'_GPS')
        self.ser = UbloxSerial(self.usb_port, self.baud_rate, self.mode, self.use_corrections)

        #region Conditional attachments to events based on rover/base
        if self.mode == 'Heading_Base':
            # Publisher to publish RTCM corrections to Rover
            self.rtcm_publisher = self.create_publisher(RtcmMessage, 'rtcm_corrections', 50)
            self.ser.rtcm_message_found += self.handle_rtcm_message
            pass
        elif self.mode == 'Rover':
            # Subscriber for receiving RTCM corrections from base.
            self.rtcm_subscriber = self.create_subscription(RtcmMessage, 'rtcm_corrections', self.handle_rtcm_message, 50)
            # Publisher for location information. Taking location from rover since it is more accurate.
            self.publisher = self.create_publisher(NavSatFix, 'gps', 50)
            # Publisher for location information. Taking location from rover since it is more accurate.
            self.status_publisher = self.create_publisher(GnssSignalStatus, 'gps_extended', 50)
            # Timer to poll status messages from base/rover for every sec.
            self.status_timer = self.create_timer(1, self.get_status)
            pass
        elif self.mode =='Disabled':
            # Publisher for location information. Taking location from rover since it is more accurate.
            self.publisher = self.create_publisher(NavSatFix, 'gps', 50)
            # Publisher for location information. Taking location from rover since it is more accurate.
            self.status_publisher = self.create_publisher(GnssSignalStatus, 'gps_extended', 50)
            # Timer to poll status messages from base/rover for every sec.
            self.status_timer = self.create_timer(1, self.get_status)
            pass
            
        # Establishing Pointperfect connection only if it's enabled. Required parameters needs to be sent.
        if self.use_corrections:
            self.pp = PointPerfectModule(self.config_path, self.region)
            # self.ser.add_to_poll('RXM', 'RXM-SPARTN-KEY') # this is needed to periodically check for keys and reconnect to pointperfect
            self.pp.on_correction_message += self.handle_correction_message
            self.reconnect_timer = self.create_timer(30, self.__reconnect_pointperfect_if_needed)
        
        #endregion

        self.lock = threading.Lock()

        # Timer to poll status messages from base/rover for every sec.
        # self.timer = self.create_timer(1, self.ser.poll)

        pass

    """
        Handles the correction messages received from PointPerfect MQTT connection.

        The decoding of correction messages and applying them to the location is done by the antenna itself.
        Need to make sure we send entire message to the antenna without a delay, Since the correction messages are time dependent.
    """
    def handle_correction_message(self, message) -> None:
        self.logger.debug(message= "Sending correction message: " + message.hex(' '))
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
                self.logger.info('Published GPS data - Latitude: {:.6f}, Longitude: {:.6f}'.format(latitude, longitude))           
        pass
    
    """
        Handles received RTCM message. 
        
        If the antenna is base, The received RTCM message is from antenna and is published to the {rtcm_topic_name} topic via ByteMultiArray message.
        If the antenna is rover, The received RTCM message is from {rtcm_topic_name} topic via ByteMultiArray message and is sent to the antenna through serial port.
    """
    def handle_rtcm_message(self, rtcmMessage) -> None:
        if self.mode == 'Heading_Base':
            msg = RtcmMessage()
            encoded_msg = base64.b64encode(rtcmMessage.serialize()).decode()
            msg.identity = rtcmMessage.identity
            msg.payload = encoded_msg
            self.rtcm_publisher.publish(msg)
            self.logger.info('Published RTCM message with identity: ' + rtcmMessage.identity) 
        else:
            data = base64.b64decode(rtcmMessage.payload.encode())
            rmg = RTCMReader.parse(data)
            self.ser.send(rmg.serialize())
            self.logger.info('Received RTCM message with identity: ' + rmg.identity)
        pass

    def get_status(self) -> None:
        status = self.ser.get_status()
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'gps'
        status.header = header
        self.status_publisher.publish(status)
        if status.latitude is not None and status.longitude is not None:
            msg = NavSatFix()
            msg.header = header
            msg.latitude = status.latitude
            msg.longitude = status.longitude
            msg.altitude = status.altitude
            msg.position_covariance = status.position_covariance
            msg.position_covariance_type = status.position_covariance_type
            msg.status = status.status
            self.publisher.publish(msg)
            self.logger.info('Published GPS data - Latitude: {:.6f}, Longitude: {:.6f}'.format(status.latitude, status.longitude))    
        pass
    
    """
        This is a continuous loop to check for any parameter updates in runtime.
    """
    def __process(self) -> None:
        while(rclpy.ok()):
            # Checking if the parameter values are changed from start.
            # serial parameters check.
            if (self.usb_port != self.get_parameter('usb_port').get_parameter_value().string_value) or (self.baud_rate != self.get_parameter('baud_rate').get_parameter_value().integer_value):
                self.logger.info("Parameters Updated. Port:"+ self.get_parameter('usb_port').get_parameter_value().string_value + ", Baud Rate: " + str(self.get_parameter('baud_rate').get_parameter_value().integer_value))
                if self.__reconfig_serial_module():
                    self.logger.info("Port reconfigured.")
                    self.usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
                    self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
                else:
                    self.logger.warn("Port not reconfigured.")
            
            if (self.save_logs != self.get_parameter('save_logs').get_parameter_value().bool_value):
                self.save_logs = self.get_parameter('save_logs').get_parameter_value().bool_value
                Logger().toggle_logs(self.save_logs)

            if (self.log_level != self.get_parameter('log_level').get_parameter_value().integer_value):
                log_level = self.get_parameter('log_level').get_parameter_value().integer_value
                if Logger().setLevel(log_level):
                    self.log_level = log_level
            time.sleep(10)
        pass
    
    """
        This method is called when serial parameters are changed in runtime. Serial module is reconfigured.
    """
    def __reconfig_serial_module(self) -> bool:
        usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        return self.ser.reconfig_serial_port(usb_port, baud_rate)

    # def __reconfig_pointperfect_module(self) -> bool:
    #     self.use_corrections = self.get_parameter("use_corrections").get_parameter_value().bool_value
    #     if self.use_corrections:
    #         self.pp = PointPerfectModule(self.config_path, self.region)
    #         self.pp.on_correction_message += self.handle_correction_message  
    #     else:
    #         self.pp.shutdown()
    #         self.pp.on_correction_message -= self.handle_correction_message
    #         self.pp = None
    #         pass
    
    """
        Spartn keys are checked in the antenna. if no keys are present, pointperfect is reconnected to get a new pair of keys.
    """
    def __reconnect_pointperfect_if_needed(self):
        if self.use_corrections:
            sptn_key = self.ser.get_recent_ubx_message('RXM-SPARTN-KEY')
            if sptn_key is not None and sptn_key.numKeys == 0:
                self.pp.reconnect()
                pass
        
def main():
    rclpy.init()
    args = rclpy.utilities.remove_ros_args(sys.argv)
    tallysman_gps = TallysmanGps(mode=args[1])
    rclpy.spin(tallysman_gps)
    tallysman_gps.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
