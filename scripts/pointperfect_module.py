import asyncio
import logging
from typing import Callable, List
import paho.mqtt.client as mqtt
import rclpy
import time

class PointPerfectModule:
    def __init__(self, logger, onCorrectionMessage):
        self.pc = PointPerfectConfiguration()
        pc = self.pc
        self.client = mqtt.Client(client_id=pc.ClientId)
        self.client.tls_set(ca_certs=pc.CaCert, certfile= pc.CertFile, keyfile= pc.KeyFile)
        self.client.on_message = onCorrectionMessage
        self.client.on_connect = self.__on_connect
        self.client.on_disconnect = self.__on_disconnect
        self.logger : logging.Logger = logging.getLogger()
        self.logger.setLevel(logging.DEBUG)
        self.client.enable_logger(logger=self.logger)
    
    def __get_client(self):
        return self.client

    def __on_message(self, client, userdata, message):
        self.logger.info("Received message '" + str(message.payload) + "' on topic '"
            + message.topic + "' with QoS " + str(message.qos))

    def __get_connection_message(self, rc: int):
        if rc == 0:
            return "Connection successful."
        elif rc == 6:
            return "Currently Unused."
        else:
            return "Connection refused."

    def __on_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.logger.warn("Unexpected disconnection.")

    def __on_connect(self, client, userdata, flags, rc):
        self.logger.info("Connection returned result: " + self.__get_connection_message(rc))

    # this should run only once for instance.
    def process(self):
        while(rclpy.ok()):
            if(not self.client.is_connected()):
                self.logger.info("PointPerfect Reconnecting....")
                try:
                    self.__connect()
                    # just to make sure connect is completed. need to change this to a better check.
                    time.sleep(2)
                    self.client.subscribe(self.pc.Topics)
                    self.client.loop_start()
                except:
                    self.logger.warn("Exception occured in pointperfect module.")

    def __connect(self):
        pc = self.pc
        self.client.connect(host=pc.Host, port=pc.Port, keepalive=pc.KeepAlive)

    def __disconnect(self):
        self.client.disconnect()
