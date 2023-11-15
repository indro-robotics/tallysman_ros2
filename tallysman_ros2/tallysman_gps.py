import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import threading
import time
from queue import Queue
import sys
from rclpy.parameter import Parameter
class TallysmanGPSPublisher(Node):
    def __init__(self):
        super().__init__('tallysman_gps_publisher')
 
        self.declare_parameter('usb_port','/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 230400)
        self.declare_parameter('topic_name', 'gps_data')
 
        usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        topic_name = self.get_parameter("topic_name").get_parameter_value().string_value

 
        self.publisher_ = self.create_publisher(NavSatFix, topic_name, 50)
        self.ser = serial.Serial(usb_port, baud_rate)
        self.lock = threading.Lock()
        self.data_queue = Queue()
 
        # Create a timer that calls the 'publish_gps_data' method every 0.1 seconds (10 Hz).
        self.timer = self.create_timer(0.05, self.publish_gps_data)
 
        # Start a separate thread for reading and parsing GPS data
        self.read_thread = threading.Thread(target=self.read_and_parse_data)
        self.read_thread.daemon = True  # Allow the thread to be terminated when the main program exits
        self.read_thread.start()
 
        
 
    def read_and_parse_data(self):
        while rclpy.ok():
            try:
                data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    self.data_queue.put(data)
            except UnicodeDecodeError:
                self.get_logger().warn('Ignoring invalid characters in received data.')
 
 
    def publish_gps_data(self):
        while not self.data_queue.empty():
            data = self.data_queue.get()
            with self.lock:
                if data.startswith('$GNGGA'):
                    latitude, longitude = parse_nmea_gga(data)
                elif data.startswith('$GNRMC'):
                    latitude, longitude = parse_nmea_rmc(data)
                else:
                    latitude, longitude = None, None
 
                if latitude is not None and longitude is not None:
                    msg = NavSatFix()
                    msg.latitude = latitude
                    msg.longitude = longitude
                    self.publisher_.publish(msg)
                    current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                    self.get_logger().info('[{}] Published GPS data - Latitude: {:.6f}, Longitude: {:.6f}'.format(current_time, latitude, longitude))
 
def parse_nmea_gga(data):
    # Your existing parse_nmea_gga function here
     
    # Split the NMEA GGA data into individual parts using comma (,) as the delimiter
    parts = data.split(',')
 
    # Check if the data contains at least 10 parts and the necessary fields for latitude and longitude extraction
    if len(parts) >= 10 and parts[2] and parts[4] and parts[9]:
     
        # Extract the latitude and longitude values from their respective parts and convert them to decimal degrees formate
        latitude = float(parts[2]) / 100
        longitude = float(parts[4]) / 100
 
        # convert to parsed lat
        degrees = int(latitude)
        minutes = 100 *(latitude -degrees) / 60
        latitude = float(degrees + minutes)
 
        # convert to parsed long
        degrees = int(longitude)
        minutes = 100* (longitude -degrees) / 60
        longitude = float(degrees + minutes)
     
        # Get the latitude and longitude directions (North/South and East/West).
        lat_direction = parts[3]
        lon_direction = parts[5]
     
        # Check if the latitude is in the Southern hemisphere and negate it if necessary.
        if lat_direction == 'S':
            latitude = -latitude
         
        # Check if the longitude is in the Western hemisphere and negate it if necessary.
        if lon_direction == 'W':
            longitude = -longitude
         
        # Return the parsed GPS latitude and longitude as a tuple
        return latitude, longitude
    else:
        # Return None for latitude and longitude if the necessary fields are not present in the data.
        return None, None
 
 
def parse_nmea_rmc(data):
    # Your existing parse_nmea_rmc function here
 
    # Split the NMEA RMC data into individual parts using comma (,) as the delimiter.
    parts = data.split(',')
 
    # Check if the data contains at least 7 parts and the necessary fields for latitude and longitude extraction.
    if len(parts) >= 7 and parts[3] and parts[5] and parts[9]:
 
        # Extract the latitude and longitude values from their respective parts and convert them to decimal degrees format.
        latitude = float(parts[3]) / 100
        longitude = float(parts[5]) / 100
 
        # convert to parsed lat
        degrees = int(latitude)
        minutes = 100 *(latitude -degrees) / 60
        latitude = float(degrees + minutes)
 
        # convert to parsed long
        degrees = int(longitude)
        minutes = 100* (longitude -degrees) / 60
        longitude = float(degrees + minutes)
        # Get the latitude and longitude directions (North/South and East/West).
        lat_direction = parts[4]
        lon_direction = parts[6]
 
        # Check if the latitude is in the Southern hemisphere and negate it if necessary.
        if lat_direction == 'S':
            latitude = -latitude
         
        # Check if the longitude is in the Western hemisphere and negate it if necessary.
        if lon_direction == 'W':
            longitude = -longitude
 
        # Return the parsed GPS latitude and longitude as a tuple.
        return latitude, longitude
 
    else:
        # Return None for latitude and longitude if the necessary fields are not present in the data.
        return None, None
 
 
def main(args=None):
    rclpy.init(args=args)
    tallysman_gps_publisher = TallysmanGPSPublisher()
    rclpy.spin(tallysman_gps_publisher)
    tallysman_gps_publisher.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
