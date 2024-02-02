import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import folium
import os
import threading
import http.server
import socketserver
import json

class GPSDataSubscriber(Node):
    def __init__(self):
        super().__init__('gps_data_subscriber')
        self.subscription = self.create_subscription(NavSatFix, 'gps', self.callback, 10)
        self.map_file = 'gps_map.html'
        self.history = []  # List to store historical GPS data points

        # Load historical GPS data from file, if available
        self.load_history()

    def load_history(self):
        if os.path.exists('gps_history.json'):
            with open('gps_history.json', 'r') as history_file:
                self.history = json.load(history_file)

    def save_history(self):
        with open('gps_history.json', 'w') as history_file:
            json.dump(self.history, history_file)

    def callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude

        if latitude != 0.0 and longitude != 0.0:  # Filter out invalid GPS data
            # Add the current GPS data point to the history
            self.history.append((latitude, longitude))

            # Save the historical GPS data to a file
            self.save_history()

            # Remove the previous map file if it exists
            if os.path.exists(self.map_file):
                os.remove(self.map_file)

            # Create a new map centered on the current GPS coordinates
            new_map = folium.Map(location=[latitude, longitude], zoom_start=10)

            # Add markers for all historical points with smaller icons
            for lat, lon in self.history:
                folium.Marker([lat, lon], icon=folium.Icon(icon='cloud', color='blue')).add_to(new_map)

            # Save the new map to an HTML file
            new_map.save(self.map_file)

def start_http_server():
    # Start a simple HTTP server to serve the map
    handler = http.server.SimpleHTTPRequestHandler
    with socketserver.TCPServer(("localhost", 8000), handler) as httpd:
        httpd.serve_forever()

def main(args=None):
    rclpy.init(args=args)
    gps_data_subscriber = GPSDataSubscriber()
    http_server_thread = threading.Thread(target=start_http_server)
    http_server_thread.daemon = True
    http_server_thread.start()
    rclpy.spin(gps_data_subscriber)
    gps_data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

