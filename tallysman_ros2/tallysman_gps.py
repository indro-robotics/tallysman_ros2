import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import time

def parse_nmea_gga(data):
 
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
 
class TallysmanGPSPublisher(Node):
    def __init__(self):
        super().__init__('tallysman_gps_publisher')

        # Create a publisher to publish NavSatFix messages on the topic 'tallysman_gps_data' with a queue size of 50.
        self.publisher_ = self.create_publisher(NavSatFix, 'tallysman_gps_data', 50)

        # Initialize a serial connection with the GPS device at '/dev/ttyUSB0' with a baud rate of 230400.
        self.ser = serial.Serial('/dev/ttyUSB0', 230400)

        # Create a timer that calls the 'publish_gps_data' method every 0.1 seconds (10 Hz).
        self.timer = self.create_timer(0.05, self.publish_gps_data)  # Publish data every 0.1 second

    def publish_gps_data(self):
        try:
            # Read a line of NMEA data from the GPS device and decode it to a string using 'utf-8' encoding,
            # ignoring any invalid characters and removing leading/trailing whitespaces.
            data = self.ser.readline().decode('utf-8', errors='ignore').strip()

            # Check if any data is received from the GPS device.
            if data:

                # Check the NMEA sentence type to determine which parser function to use.
                if data.startswith('$GNGGA'):
                    # Parse the NMEA GGA data to extract GPS latitude and longitude.
                    latitude, longitude = parse_nmea_gga(data)
                 
                elif data.startswith('$GNRMC'):
                    # Parse the NMEA RMC data to extract GPS latitude and longitude.
                    latitude, longitude = parse_nmea_rmc(data)
                 
                else:
                    # If the sentence type is unknown or unsupported, set latitude and longitude to None.
                    latitude, longitude = None, None

                # Check if valid latitude and longitude values are obtained.
                if latitude is not None and longitude is not None:

                    # Create a NavSatFix message and set its latitude and longitude fields.
                    msg = NavSatFix()
                    msg.latitude = latitude
                    msg.longitude = longitude

                    # Publish the NavSatFix message on the 'tallysman_gps_data' topic.
                    self.publisher_.publish(msg)

                    # Get the current time in human-readable format
                    current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                 
                    # Log the published GPS data along with the current time.
                    self.get_logger().info('[{}] Published GPS data - Latitude: {:.6f}, Longitude: {:.6f}'.format(current_time, latitude, longitude))
         
        except UnicodeDecodeError:
            # If there is a UnicodeDecodeError while decoding the received data, log a warning indicating that invalid characters are ignored.
            self.get_logger().warn('Ignoring invalid characters in received data.')

def main(args=None):
    rclpy.init(args=args)

    tallysman_gps_publisher = TallysmanGPSPublisher()

    rclpy.spin(tallysman_gps_publisher)

    tallysman_gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()