from enum import Enum
import threading
from typing import overload
import serial
from events import Event
import rclpy
from pyubx2 import ubxreader, UBXMessage, GET, SET, POLL
from pynmeagps import NMEAMessage
from pyrtcm import RTCMMessage

class UbloxSerial:
    
    def __init__(self, portName: str, baudrate: int, logger):
        self.__port = serial.Serial(portName, baudrate)
        self.ubr = ubxreader.UBXReader(self.__port, protfilter=7)

        self.logger = logger
        
        self.ublox_message_found: Event[UBXMessage] = Event() 
        self.nmea_message_found: Event[NMEAMessage] = Event()
        self.rtcm_message_found: Event[RTCMMessage] = Event()
    
        # Start a separate thread for reading and parsing GPS data
        self.read_thread = threading.Thread(target=self.__receive_thread, name="receive_thread_"+portName)
        self.read_thread.daemon = True  # Allow the thread to be terminated when the main program exits
        self.read_thread.start()
        self.config()
        pass
    
    def open(self) -> None:
        if self.__port.is_open:
            pass
        else:
            self.__port.open()

    def close(self) -> None:
        if self.__port.is_open:
            self.__port.close()

    def __receive_thread(self) -> None:
        while(rclpy.ok()):
            if self.__port.is_open:
                try:
                    for (raw_data, parsed_data) in self.ubr:
                        if isinstance(parsed_data, NMEAMessage):
                            self.nmea_message_received(parsed_data)
                        if isinstance(parsed_data, UBXMessage):
                            self.ublox_message_received(parsed_data)
                        if isinstance(parsed_data, RTCMMessage):
                            self.rtcm_message_received(parsed_data)
                except :
                    self.logger.warn("Exception occured")
                    pass
            else:
                self.open()
                self.logger.warn("Reconnecting to port.")

    def ublox_message_received(self, message: UBXMessage) -> None:
        self.logger.info("[Ublox]: "+ message.identity)
        self.ublox_message_found(message)
        pass

    def nmea_message_received(self, message: NMEAMessage) -> None:
        if message.identity == "GNRMC" or message.identity == "GNGGA":
            self.logger.info("[Nmea]: "+ message.identity)
            self.nmea_message_found(message)
        pass

    def rtcm_message_received(self, message: RTCMMessage) -> None:
        self.logger.info("[RTCM]: "+ message.identity)
        self.rtcm_message_found(message)
        pass

    def send(self, data: bytes) -> None:
        if self.__port.is_open:
            self.__port.write(data)
        else:
            raise Exception("Port not open")
    
    def poll_messages(self):
        ubx = UBXMessage('NAV', 'NAV-POSECEF', POLL)
        self.send(ubx.serialize())
        pass

    def config(self):
        configData: list = [('CFG_SPARTN_USE_SOURCE', 0), ('CFG_SPARTN_USE_SOURCE', 1)]
        ubx = UBXMessage.config_set(1, 0, configData)
        self.send(ubx.serialize())