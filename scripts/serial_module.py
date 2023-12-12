import logging
import threading
import time
from typing import Literal, overload
import serial
from events import Event
import rclpy
from pyubx2 import ubxreader, UBXMessage, GET, SET, POLL
from pynmeagps import NMEAMessage
from pyrtcm import RTCMMessage
from tallysman_ros2_msgs.msg import GnssSignalStatus, GnssLocation
from builtin_interfaces.msg import Time

class UbloxSerial:
    """
        Serial module to handle read and write operations for the antenna.

        port_name: Bidirectional port through which node communicates with antenna.
        baudrate: The speed at which data is transmitted.
        logger: Logger must contain basic log functions.
    """
    def __init__(self, port_name: str, baudrate: Literal[1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800], logger, is_base, use_corrections):
        self.__port = serial.Serial(port_name, baudrate)
        
        self.__is_base = is_base
        self.__use_corrections = use_corrections

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

        # Configurations to antenna to work in a specified mode.
        self.config()
        self.__recent_ubx_message = dict[str,(float,UBXMessage)]()

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
                    raise
            else:
                self.open()
                self.__logger.warn("Reconnecting to port.")

    """
        Message handler for Ublox message. Invokes ublox_message_found event.
    """
    def ublox_message_received(self, message: UBXMessage) -> None:
        self.__logger.info("[Ublox]: "+ message.identity)
        self.__recent_ubx_message[message.identity] = (time.time(), message)
        self.ublox_message_found(message)
        pass
    
    """
        Message handler for Nmea message. Invokes nmea_message_found event.
    """
    def nmea_message_received(self, message: NMEAMessage) -> None:
        self.__logger.info("[Nmea]: "+ message.identity)
        if message.identity == "GNRMC" or message.identity == "GNGGA":
            self.__latitude = message.lat if message.lat else None
            self.__longitude = message.lon if message.lon else None
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
        poll_messages = [('NAV','NAV-HPPOSECEF'), ('NAV', 'NAV-HPPOSLLH'), ('NAV', 'NAV-PVT')]
        if not self.__is_base:
            poll_messages.append(('NAV', 'NAV-RELPOSNED'))

        for (class_name, msg_name) in poll_messages:
            ubx = UBXMessage(class_name, msg_name, POLL)
            self.send(ubx.serialize())
        pass
    
    """
        Configures the antenna based on parameters.
        Refer ubxtypes_configdb.py at /pyubx2/ubxtypes_configdb.py for equivalent name strings for different keys
    """
    def config(self):
        configData: list
        configData = [('CFG_UART1INPROT_NMEA',1), ('CFG_UART1INPROT_UBX',1), ('CFG_UART1OUTPROT_NMEA',1), ('CFG_UART1OUTPROT_UBX',1)] # common configurations
        if self.__is_base:
            # base related configurations
            configData.extend([('CFG_UART1INPROT_RTCM3X', 0), ('CFG_UART1OUTPROT_RTCM3X', 1), ('CFG_NAVSPG_DYNMODEL', 2)]) 
            configData.extend([('CFG_MSGOUT_RTCM_3X_TYPE1074_UART1', 0x1), ('CFG_MSGOUT_RTCM_3X_TYPE1084_UART1', 0x1), ('CFG_MSGOUT_RTCM_3X_TYPE1124_UART1', 0x1), ('CFG_MSGOUT_RTCM_3X_TYPE1094_UART1', 0x1)])

            # base heading configurations
            configData.extend([('CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1', 0x1), ('CFG_MSGOUT_RTCM_3X_TYPE1230_UART1', 0x1), ('CFG_TMODE_MODE', 0x0)])

            # pointperfect related configurations
            if self.__use_corrections:
                configData.extend([('CFG_SPARTN_USE_SOURCE',0), ('CFG_UART1INPROT_SPARTN',1), ('CFG_MSGOUT_UBX_RXM_SPARTN_UART1',1)])
        else:
            # rover related configurations
            configData.extend([('CFG_UART1INPROT_RTCM3X', 1), ('CFG_MSGOUT_UBX_RXM_RTCM_UART1', 0x1)]) 
        
        # Sets the configs only to RAM. will reset if the antenna is power cycled.
        ubx: UBXMessage = UBXMessage.config_set(4, 0, configData)
        self.send(ubx.serialize())
        ubx = UBXMessage.config_set(1, 0, configData)
        self.send(ubx.serialize())

    """
        returns the Rover/Base status. Currently returning only Rover status. Will improve this further.
    """
    def get_status(self) -> GnssSignalStatus :
        gnss_status = GnssSignalStatus()
        loc = GnssLocation()
        try:
            
            t = Time()
            t.sec = int(time.time())
            loc.time = t
            if self.__latitude is not None and self.__longitude is not None:
                loc.latitude = float(self.__latitude) 
                loc.longitude = float(self.__longitude)
                loc.valid_fix = True
            else:
                loc.valid_fix = False
            gnss_status.gnss_location = loc
            nav_hpposllh = self.get_recent_ubx_message('NAV-HPPOSLLH')
            nav_pvt = self.get_recent_ubx_message('NAV-PVT')
            nav_hpposecef = self.get_recent_ubx_message('NAV-HPPOSECEF')
            nav_relposned = self.get_recent_ubx_message('NAV-RELPOSNED')
            rxm_rtcm = self.get_recent_ubx_message('RXM-RTCM')
            #gnss_status.header.time.sec = int(time.time())
            if nav_relposned is not None:
                gnss_status.heading = nav_relposned.relPosHeading
                gnss_status.length = float(nav_relposned.relPosLength)
            if nav_hpposecef is not None and nav_hpposllh is not None:
                gnss_status.two_dimension_accuracy = float(nav_hpposllh.hAcc/1000) # scaling and meters conversion
                gnss_status.three_dimension_accuracy = float(nav_hpposecef.pAcc/1000) # scaling and meters conversion
            if rxm_rtcm is not None:
                gnss_status.augmentations_used = True if rxm_rtcm.msgUsed == 2 else False
            if nav_pvt is not None:
                gnss_status.quality = self.__get_quality_string(nav_pvt)
        except:
            pass        
        return gnss_status

    """
        returns a Ubx message only if it arrived in less than 10sec
    """
    def get_recent_ubx_message(self, msgId) -> UBXMessage:
        try:
            time_of_message,ubxmessage = self.__recent_ubx_message[msgId]
            if time.time() - time_of_message < 10: # return only if received in less than 10sec
                return ubxmessage
            else:
                return None
        except:
            return None
    
    """
        returns the Quality string for status based on Nav_Pvt message.
    """
    def __get_quality_string(self, nav_pvt: UBXMessage) -> str:
        if nav_pvt is None:
            return ''
        
        quality = ''
        
        # Fix type definition
        if nav_pvt.fixType == 0:
            quality += 'NoFix'
        elif nav_pvt.fixType == 1:
            quality += 'DR'
        elif nav_pvt.fixType == 2:
            quality += '2D'
        elif nav_pvt.fixType == 3:
            quality += '3D'
        elif nav_pvt.fixType == 4:
            quality += 'GDR'
        elif nav_pvt.fixType == 5:
            quality += 'TF'

        if nav_pvt.gnssFixOk == 1:
            quality += '/DGNSS'
        
        if nav_pvt.carrSoln == 1:
            quality += '/Float'
        elif nav_pvt.carrSoln == 2:
            quality += '/Fixed'

        return quality