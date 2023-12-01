import logging
import threading
from typing import Literal, overload
import serial
from events import Event
import rclpy
from pyubx2 import ubxreader, UBXMessage, GET, SET, POLL
from pynmeagps import NMEAMessage
from pyrtcm import RTCMMessage

class UbloxSerial:
    """
        Serial module to handle read and write operations for the antenna.

        port_name: Bidirectional port through which node communicates with antenna.
        baudrate: The speed at which data is transmitted.
        logger: Logger must contain basic log functions.
    """
    def __init__(self, port_name: str, baudrate: Literal[1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800], logger):
        self.__port = serial.Serial(port_name, baudrate)
        
        # protfilter=7 gives out UBX, NMEA and RTCM messages.
        self.__ubr = ubxreader.UBXReader(self.__port, protfilter=7)

        self.__logger = logger if logger else logging.getLogger()
        
        # Event handlers for Ublox, Nmea and RTCM messages.
        self.ublox_message_found: Event[UBXMessage] = Event() 
        self.nmea_message_found: Event[NMEAMessage] = Event()
        self.rtcm_message_found: Event[RTCMMessage] = Event()
    
        # Start a separate thread for reading and parsing serial stream.
        self.__read_thread = threading.Thread(target=self.__receive_thread, name="receive_thread_"+port_name)
        self.__read_thread.daemon = True  # Allow the thread to be terminated when the main program exits
        self.__read_thread.start()
        pass
    
    def open(self) -> None:
        if self.__port.is_open:
            pass
        else:
            self.__port.open()

    def close(self) -> None:
        if self.__port.is_open:
            self.__port.close()
    
    """
        Reads data from serial port and calls respective message handlers methods.
    """
    def __receive_thread(self) -> None:
        while(rclpy.ok()):
            if self.__port.is_open:
                try:
                    for (raw_data, parsed_data) in self.__ubr:
                        if isinstance(parsed_data, NMEAMessage):
                            self.nmea_message_received(parsed_data)
                        if isinstance(parsed_data, UBXMessage):
                            self.ublox_message_received(parsed_data)
                        if isinstance(parsed_data, RTCMMessage):
                            self.rtcm_message_received(parsed_data)
                except :
                    self.__logger.warn("Exception occured")
                    pass
            else:
                self.open()
                self.__logger.warn("Reconnecting to port.")

    """
        Message handler for Ublox message. Invokes ublox_message_found event.
    """
    def ublox_message_received(self, message: UBXMessage) -> None:
        self.__logger.info("[Ublox]: "+ message.identity)
        self.ublox_message_found(message)
        pass
    
    """
        Message handler for Nmea message. Invokes nmea_message_found event.
    """
    def nmea_message_received(self, message: NMEAMessage) -> None:
        self.__logger.info("[Nmea]: "+ message.identity)
        self.nmea_message_found(message)
        pass
    
    """
        Message handler for Rtcm message. Invokes rtcm_message_found event.
    """
    def rtcm_message_received(self, message: RTCMMessage) -> None:
        self.__logger.info("[RTCM]: "+ message.identity)
        self.rtcm_message_found(message)
        pass
    
    """
        Writes data to port if the port is open. Raises an exception if not.
    """
    def send(self, data: bytes) -> None:
        if self.__port.is_open:
            self.__port.write(data)
        else:
            raise Exception("Port not open")
    
    # still need to work on this method
    """
        Polls the respective messages for rover/base to update status.
    """
    def poll_messages(self):
        ubx = UBXMessage('NAV', 'NAV-POSECEF', POLL)
        self.send(ubx.serialize())
        pass
    
    """
        Configures the antenna based on parameters.
        Refer ubxtypes_configdb.py at /pyubx2/ubxtypes_configdb.py for equivalent name strings for different keys
    """
    def config(self, is_base: bool = True, use_corrections: bool = False):
        configData: list
        configData = [('CFG_UART1INPROT_NMEA',1), ('CFG_UART1INPROT_UBX',1), ('CFG_UART1OUTPROT_NMEA',1), ('CFG_UART1OUTPROT_UBX',1)] # common configurations
        if is_base:
            # base related configurations
            configData.extend([('CFG_UART1INPROT_RTCM3X', 0), ('CFG_UART1OUTPROT_RTCM3X', 1), ('CFG_NAVSPG_DYNMODEL', 2)]) 
            configData.extend([('CFG_MSGOUT_RTCM_3X_TYPE1074_UART1', 0x1), ('CFG_MSGOUT_RTCM_3X_TYPE1084_UART1', 0x1), ('CFG_MSGOUT_RTCM_3X_TYPE1124_UART1', 0x1), ('CFG_MSGOUT_RTCM_3X_TYPE1094_UART1', 0x1)])

            # base heading configurations
            configData.extend([('CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART1', 0x1), ('CFG_MSGOUT_RTCM_3X_TYPE1230_UART1', 0x1), ('CFG_TMODE_MODE', 0x0)])

            # pointperfect related configurations
            if use_corrections:
                configData.extend([('CFG_SPARTN_USE_SOURCE',0), ('CFG_UART1INPROT_SPARTN',1), ('CFG_MSGOUT_UBX_RXM_SPARTN_UART1',1), ('CFG_UART2_BAUDRATE',460800)])
        else:
            # rover related configurations
            configData.extend([('CFG_UART1INPROT_RTCM3X', 1), ('CFG_MSGOUT_UBX_RXM_RTCM_UART1', 0x1)]) 
        
        # Sets the configs only to RAM. will reset if the antenna is power cycled.
        ubx: UBXMessage = UBXMessage.config_set(1, 0, configData)
        self.send(ubx.serialize())