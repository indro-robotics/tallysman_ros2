import logging
import os
from textwrap import wrap
import threading
from typing import Literal
import paho.mqtt.client as mqtt
import rclpy
import time
from events import Event
import json

"""
    This class is responsible to get all the fields required to make an MQTT connection for the PointPerfect correction messages.

    Needs a valid subscription to Ublox/PointPerfect corrections.
"""
class PointPerfectConfiguration:
    
    """
        region: Select based on region of operation.
        configPath: Absolute path to the ucenter-config file downloaded from https://portal.thingstream.io/
    """
    def __init__(self, region: Literal["us", "kr", "eu", "au"], configPath: str) -> None:
        self.__region = region
        self.__config_path = configPath
        self.Host: str = "pp.services.u-blox.com"
        self.Port: int = 8883
        self.__get_configs_from_file()
        pass
    
    """
        Extracts configurations from PointPerfect configuration file. Config file should be the downloaded ucenter_config_file from https://portal.thingstream.io/. 
    """
    def __get_configs_from_file(self):
        with open(self.__config_path) as json_file:
            config_json = json.load(json_file)['MQTT']
            self.KeepAlive = config_json['Connectivity']['KeepAliveInterval']
            self.ClientId = config_json['Connectivity']['ClientID']
            self.Topics = self.__get_topics(config_json['Subscriptions'])
            self.CertFile = self.__generate_cert_file("CERTIFICATE", config_json['Connectivity']['ClientCredentials']["Cert"], "pp_cert.crt")
            self.CaCert = self.__generate_cert_file("CERTIFICATE", config_json['Connectivity']['ClientCredentials']["RootCA"], "pp_ca_cert.crt")
            self.KeyFile = self.__generate_cert_file("RSA PRIVATE KEY", config_json['Connectivity']['ClientCredentials']["Key"], "pp_key.pem")   
        pass
    
    """
        Extracts necessary topics from Subsciption topics.
    """
    def __get_topics(self,subs: dict) -> None:
        topics: list[(str,int)] = [(subs['Key']['KeyTopics'][0],subs['Key']['QoS'])]
        qos = subs['Data']['QoS']
        for topic in subs['Data']['DataTopics']:
            if self.__region in topic:
                topics.append((topic, qos))
        topics.pop() # vomitting deserialized topic
        return topics
    
    """
        Generates the required cert/key files from the Configuration.
    """
    def __generate_cert_file(self, header: str, contents: str, file_name: str) -> str:
        directory = '/root/humble_ws/src/tallysman_ros2/pointperfect_files'
        with open(os.path.join(directory, file_name), "w") as file:
            lines = []
            pem_prefix = '-----BEGIN {}-----\n'.format(header)
            lines.append(pem_prefix)
            for line in wrap(contents, 64):
                lines.append(line+'\n')
            pem_suffix = '-----END {}-----'.format(header)
            lines.append(pem_suffix)
            file.writelines(lines)
            pass
        return os.path.join(directory, file_name)

class PointPerfectModule:
    def __init__(self, logger, configPath: str, region: str) -> None:
        self.__pc = PointPerfectConfiguration(region, configPath)
        self.on_correction_message: Event[bytes] = Event()

        # Mqtt client initialization.
        self.__client = mqtt.Client(client_id=self.__pc.ClientId, reconnect_on_failure=False)
        self.__client.tls_set(ca_certs=self.__pc.CaCert, certfile= self.__pc.CertFile, keyfile= self.__pc.KeyFile)

        self.__client.on_message = self.__on_message
        self.__client.on_disconnect = self.__on_disconnect
        self.__client.on_connect = self.__on_connect

        # Still need to work on logging. Ignore below lines
        self.logger : logging.Logger = logging.getLogger()
        self.logger.setLevel(logging.DEBUG)
        self.__client.enable_logger(logger=self.logger)

        # Starting a separate pointperfect thread for pointperfect process
        self.__pp_thread = threading.Thread(target=self.__process, name='corrections_thread')
        self.__pp_thread.daemon = True
        self.__pp_thread.start()
        
        pass
    
    """
        Default callback for incoming message received from MQTT connection. Invokes on_correction_message event
    """
    def __on_message(self, client, userdata, message) -> None:
        self.logger.info("Received message '" + str(message.payload) + "' on topic '"
            + message.topic + "' with QoS " + str(message.qos))
        self.on_correction_message(message.payload)

    """
        Gets connection message based on 'rc' value.
    """
    def __get_connection_message(self, rc: int) -> str:
        if rc == 0:
            return "Connection successful."
        elif rc == 6:
            return "Currently Unused."
        else:
            return "Connection refused."

    """
        Callback for MQTT disconnect.
    """
    def __on_disconnect(self, client, userdata, rc) -> None:
        if rc != 0:
            self.logger.warn("Unexpected disconnection.")

    """
        Callback for MQTT connect.
    """
    def __on_connect(self, client, userdata, flags, rc) -> None:
        self.logger.info("Connection returned result: " + self.__get_connection_message(rc))

    """
        This should run only once for instance.
        
        Attemps the MQTT connection and Subscribes to topics.
        Reconnects if the connection is lost.
    """
    def __process(self) -> None:
        while(rclpy.ok()):
            if(not self.__client.is_connected()):
                self.logger.info("PointPerfect Reconnecting....")
                try:
                    self.__connect()
                    # just to make sure connect is completed. need to change this to a better check.
                    self.__client.subscribe(self.__pc.Topics)
                    self.__client.loop_start()
                except:
                    self.logger.warn("Exception occured in pointperfect module.")
            time.sleep(0.5)

    """
        Connects to MQTT client.
    """
    def __connect(self) -> None:
        pc = self.__pc
        self.__client.connect(host=pc.Host, port=pc.Port, keepalive=pc.KeepAlive)

    """
        Disconnects from MQTT client.
    """
    def __disconnect(self) -> None:
        self.__client.disconnect()
