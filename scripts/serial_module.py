from collections import UserList
import threading
import time
from typing import Literal, overload
import serial
from events import Event
import rclpy
from pyubx2 import ubxreader, UBXMessage, GET, SET, POLL
from pynmeagps import NMEAMessage
from pyrtcm import RTCMMessage
from tallysman_msg.msg import GnssSignalStatus, GnssLocation
from sensor_msgs.msg import NavSatStatus
from scripts.logging import SimplifiedLogger

class UbloxSerial:
    """
        Serial module to handle read and write operations for the antenna.

        port_name: Bidirectional port through which node communicates with antenna.
        baudrate: The speed at which data is transmitted.
        logger: Logger must contain basic log functions.
    """
    def __init__(self, port_name: str, baudrate: Literal[1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800], rtk_mode: Literal['Disabled', 'Heading_Base', 'Rover'], use_corrections: bool = False):
        self.logger = SimplifiedLogger(rtk_mode+'_Serial')
        # Event handlers for Ublox, Nmea and RTCM messages.
        self.ublox_message_found: Event[UBXMessage] = Event() 
        self.nmea_message_found: Event[NMEAMessage] = Event()
        self.rtcm_message_found: Event[RTCMMessage] = Event()
        self.__is_running = True
        self.__rtk_mode = rtk_mode
        self.__use_corrections = use_corrections
        self.__recent_ubx_message = dict[str,(float,UBXMessage)]()
        self.__setup_serial_port_and_reader(port_name, baudrate)
        self.runTime = 0
        # self.__poll_messages: set[tuple[str,str]] = set()
        self.__process =  threading.Thread(target=self.__serial_process, name="serial_process_thread", daemon=True)
        self.__process.start()

    """
        This is continuosly running loop to make sure everything is working fine in serial module.

        This will restart the receive thread if by any case it stops due to exception.
        This will also detect if the antenna is rebooted and does the configuration to the antenna again.
    """
    def __serial_process(self) -> None:
        while(rclpy.ok()):
            if self.__is_running:
                if self.__port is not None and self.__port.is_open:
                    if self.__read_thread is not None and self.__read_thread.is_alive():
                        pass
                    else:
                        self.__read_thread = threading.Thread(target=self.__receive_thread, name="receive_thread_"+self.__port.name, daemon=True)
                        self.__read_thread.start()

                    if not self.__config_status:
                        self.__setup_serial_port_and_reader(self.port_name, self.baudrate)
                else:
                    # if self.__port is None:
                    #     self.__port = serial.Serial(self.port_name, self.baudrate)
                    if self.__port is not None and not self.__port.is_open:
                        self.__port.open()

                monSys = self.get_recent_ubx_message('MON-SYS')
                if monSys is not None:
                    if self.runTime <= monSys.runTime:
                        self.runTime = monSys.runTime
                    else:
                        self.logger.warn("Antenna rebooted. Reconfiguring the antenna")
                        # self.__save_boot_times(self.runTime)
                        self.runTime = 0
                        self.config()
                
            time.sleep(1)

    """
        This method is responsible for setting up the serial port based on port_name and baudrate. It also starts a receive_thread specific to port_name serial port and configures a reader to parse the data
    """
    def __setup_serial_port_and_reader(self, port_name: str, baudrate: Literal[1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800]) -> None:
        try:
            self.__port = serial.Serial(port_name, baudrate)
            self.port_name = port_name
            self.baudrate = baudrate
            # protfilter=7 gives out UBX, NMEA and RTCM messages.
            self.__ubr = ubxreader.UBXReader(self.__port, protfilter=7)
            
            # Start a separate thread for reading and parsing serial stream.
            self.__read_thread = threading.Thread(target=self.__receive_thread, name="receive_thread_"+port_name, daemon=True)
            self.__read_thread.start()
            self.__is_running = True
            # Configurations to antenna to work in a specified mode.
            self.config()
            pass
        except serial.SerialException as se:
            self.logger.error(se.strerror)
            self.__port = None
            self.__is_running = False
            pass
        except Exception:
            self.logger.error('Exception occured while setting up the Serial Module.')
            self.__is_running = False
        pass
    
    def open(self) -> None:
        if self.__port is not None and self.__port.is_open:
            pass
        else:
            self.__port.open()

    def close(self) -> None:
        if self.__port is not None and self.__port.is_open:
            self.__is_running = False
            self.__port.close()
    
    """
        This method is intended to reconfigure the serial port. This should only be used if the dynamic parameters like port_name or baudrate are changed after initialization.
    """
    def reconfig_serial_port(self, port_name: str, baudrate: Literal[1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800]) -> bool:
        try:
            self.close()
            self.__ubr = None
            # Configurations to antenna to work in a specified mode.
            self.__recent_ubx_message.clear()
            self.__is_running = False
            self.__setup_serial_port_and_reader(port_name, baudrate)
        except:
            self.logger.error("Exception occured while reconfiguring the port.")
            self.__is_running = False
            return False
        
        return True

    # def reconfig_pointperfect_config(self, use_corrections) -> None:
    #     self.__use_corrections = use_corrections
    #     self.config()

    """
        Reads data from serial port and calls respective message handlers methods.
    """
    def __receive_thread(self) -> None:
        while(rclpy.ok()):
            if self.__is_running:
                if self.__port is not None and self.__port.is_open:
                    try:
                        (raw_data, parsed_data) = self.__ubr.read()
                        if isinstance(parsed_data, NMEAMessage):
                            self.__nmea_message_received(parsed_data)
                        elif isinstance(parsed_data, UBXMessage):
                            self.__ublox_message_received(parsed_data)
                        elif isinstance(parsed_data, RTCMMessage):
                            self.__rtcm_message_received(parsed_data)
                    except Exception as e:
                        self.logger.warn("Port is open but unable to read from port. Error : {error}".format(error=str(e)))
                        break
                else:
                    self.logger.warn("Port is not configured/open.")
                    break
            else:
                break

    """
        Message handler for Ublox message. Invokes ublox_message_found event.
    """
    def __ublox_message_received(self, message: UBXMessage) -> None:
        self.logger.debug(" <- [Ubx:{identity}] : {bytes}".format(identity = message.identity, bytes = str(message)))
        if ((message.identity == "RXM-SPARTN" or message.identity == "RXM-RTCM") and message.msgUsed == 2) or (message.identity != "RXM-SPARTN" and message.identity != "RXM-RTCM"):
            self.__recent_ubx_message[message.identity] = (time.time(), message)
        self.ublox_message_found(message)
        pass
    
    """
        Message handler for Nmea message. Invokes nmea_message_found event.
    """
    def __nmea_message_received(self, message: NMEAMessage) -> None:
        self.logger.debug(" <- [Nmea:{identity}] : {bytes}".format(identity = message.identity, bytes = str(message)))
        if message.identity == "GNRMC" or message.identity == "GNGGA":
            self.__latitude = message.lat if message.lat else None
            self.__longitude = message.lon if message.lon else None
        self.nmea_message_found(message)
        pass
    
    """
        Message handler for Rtcm message. Invokes rtcm_message_found event.
    """
    def __rtcm_message_received(self, message: RTCMMessage) -> None:
        self.logger.debug(" <- [Rtcm:{identity}] : {bytes}".format(identity = message.identity, bytes = str(message)))
        self.rtcm_message_found(message)
        pass
    
    """
        Writes data to port if the port is open. Raises an exception if not.
    """
    def send(self, data: bytes) -> None:
        try:
            if self.__port is not None and self.__port.is_open:
                self.logger.debug(" -> {bytes}".format(bytes=data.hex(' ')))
                self.__port.write(data)
            else:
                self.logger.warn("Port is not configured/open.")
        except Exception:
            pass
    
    # still need to work on this method
    """
        Polls the respective messages for rover/base to update status.
    """
    def poll(self):
        if self.__port is not None:
            self.logger.debug(" -> {bytes}".format(bytes="Polling Messages"))
            for (class_name, msg_name) in self.__poll_messages:
                ubx = UBXMessage(class_name, msg_name, POLL)
                self.send(ubx.serialize())
        else:
            self.logger.warn('Port is not configured/open.')
        pass
    
    def poll_once(self, class_name: str, msg_name: str)-> UBXMessage:
        ubx = UBXMessage(class_name, msg_name, POLL)
        retry_count = 0
        polled_message:UBXMessage = None
        while(polled_message is None and retry_count < 3):
            self.send(ubx.serialize())
            time.sleep(0.5) # delay to make sure antenna response is logged before another attempt.
            polled_message = self.get_recent_ubx_message(msg_name)
            retry_count = retry_count + 1
        return polled_message

    """
        Adds only if not present.
    """
    def add_to_poll(self, class_name: str, msg_name: str)->None:
        self.__poll_messages.add((class_name, msg_name))
        pass
    
    """
        Adds only if not present and deletes immediately so it is removed if present or not present from the set. Update if better method is found.
    """
    def remove_from_poll(self, class_name: str, msg_name: str)->None:
        self.__poll_messages.add((class_name, msg_name))
        self.__poll_messages.remove((class_name, msg_name))
        pass

    """
        Configures the antenna based on parameters. Retries the config for 3times before throwing the error.
        Refer ubxtypes_configdb.py at /pyubx2/ubxtypes_configdb.py for equivalent name strings for different keys
    """
    def config(self) -> bool:
        config_successful = False
        config_data = self.__get_config_set(mode_of_operation=self.__rtk_mode, use_corrections=self.__use_corrections)
        
        # Sets the configs only to RAM. will reset if the antenna is power cycled.
        ubx: UBXMessage = UBXMessage.config_set(1, 0, config_data)
        retry_count = 0
        while(not config_successful and retry_count < 3):
            self.send(ubx.serialize())
            time.sleep(0.5) # delay to make sure antenna response is logged before another attempt.
            if self.get_recent_ubx_message('ACK-ACK') is not None:
                config_successful = True
            retry_count = retry_count + 1
        
        self.__config_status = config_successful
        if not self.__config_status:
            self.logger.error("Configuration failed.")
        return config_successful

    """
        returns the Rover/Base status. Currently returning only Rover status. Will improve this further.
    """
    def get_status(self) -> GnssSignalStatus :
        gnss_status = GnssSignalStatus()
        try:
            if self.__latitude is not None and self.__longitude is not None:
                gnss_status.latitude = float(self.__latitude) 
                gnss_status.longitude = float(self.__longitude)
                gnss_status.valid_fix = True
            else:
                gnss_status.valid_fix = False
            # gnss_status.gnss_location = loc
            nav_hpposllh = self.get_recent_ubx_message('NAV-HPPOSLLH')
            nav_pvt = self.get_recent_ubx_message('NAV-PVT')
            nav_relposned = self.get_recent_ubx_message('NAV-RELPOSNED')
            nav_hpposecef = self.get_recent_ubx_message('NAV-HPPOSECEF')
            augmentations_used = False
            if self.__rtk_mode == 'Disabled':
                rxm_spartn = self.get_recent_ubx_message('RXM-SPARTN')
                augmentations_used = True if rxm_spartn and rxm_spartn.msgUsed == 2 else False
            else: 
                rxm_rtcm = self.get_recent_ubx_message('RXM-RTCM')
                augmentations_used = True if rxm_rtcm and rxm_rtcm.msgUsed == 2 else False
            #gnss_status.header.time.sec = int(time.time())
            if nav_relposned is not None:
                gnss_status.heading = nav_relposned.relPosHeading
                gnss_status.length = float(nav_relposned.relPosLength)
            if nav_hpposecef is not None and nav_hpposllh is not None:
                variance = [float(nav_hpposllh.hAcc**2),0.0,0.0,0.0,float(nav_hpposllh.vAcc**2),0.0,0.0,0.0,float(nav_hpposllh.hAcc**2)]
                gnss_status.position_covariance = UserList(variance)
                gnss_status.position_covariance_type = gnss_status.COVARIANCE_TYPE_DIAGONAL_KNOWN
                gnss_status.altitude = float(nav_hpposllh.height/1000)#scaling and meters conversion
                gnss_status.two_dimension_accuracy = float(nav_hpposllh.hAcc/1000) # scaling and meters conversion
                gnss_status.three_dimension_accuracy = float(nav_hpposecef.pAcc/1000) # scaling and meters conversion
            gnss_status.augmentations_used = augmentations_used
            if nav_pvt is not None:
                status = NavSatStatus()
                if nav_pvt.gnssFixOk == 1:
                    if nav_pvt.carrSoln != 0:
                        status.status = NavSatStatus.STATUS_GBAS_FIX
                    else:
                        status.status = NavSatStatus.STATUS_FIX
                else:
                    status.status = NavSatStatus.STATUS_NO_FIX
                
                status.service = NavSatStatus.SERVICE_GALILEO | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_COMPASS
                gnss_status.status = status
                gnss_status.quality = self.__get_quality_string(nav_pvt)
        except Exception as ex:
            print(ex)
            pass        
        return gnss_status

    """
        returns a Ubx message only if it arrived in the last 10sec.
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
    
    def __get_config_set(self, mode_of_operation: Literal['Disabled', 'Heading_Base', 'Rover'], use_corrections: bool = False) -> list :
        # Common configuration. Enabling Nmea, Ubx messages for both input and output.
        config_data = [('CFG_UART1INPROT_NMEA', 1), ('CFG_UART1INPROT_UBX', 1), ('CFG_UART1OUTPROT_NMEA', 1), ('CFG_UART1OUTPROT_UBX', 1), ('CFG_MSGOUT_UBX_MON_SYS_UART1', 1), ('CFG_NAVSPG_DYNMODEL', 0)]

        if mode_of_operation == 'Disabled':
            config_data.extend([('CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART1', 1), ('CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART1', 1), ('CFG_MSGOUT_UBX_NAV_PVT_UART1', 1)])
            pass
        elif mode_of_operation == 'Heading_Base':
            # Enabling output RTCM and disabling input RTCM
            config_data.extend([('CFG_UART1INPROT_RTCM3X', 0), ('CFG_UART1OUTPROT_RTCM3X', 1)]) 

            # Common RTCM message types for Base (1074, 1084, 1094, 1124).
            config_data.extend([('CFG_MSGOUT_RTCM_3X_TYPE1074_UART1', 0x1), ('CFG_MSGOUT_RTCM_3X_TYPE1084_UART1', 0x1), ('CFG_MSGOUT_RTCM_3X_TYPE1124_UART1', 0x1), ('CFG_MSGOUT_RTCM_3X_TYPE1094_UART1', 0x1)])

            # 4072.0, 1230 is Enabled for Heading_Base. Also, TimeMode is set to disabled.
            config_data.extend([('CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1', 0x1), ('CFG_MSGOUT_RTCM_3X_TYPE1230_UART1', 0x1), ('CFG_TMODE_MODE', 0x0)])
        elif mode_of_operation == 'Rover':
            # rover related configurations
            config_data.extend([('CFG_UART1INPROT_RTCM3X', 1), ('CFG_MSGOUT_UBX_RXM_RTCM_UART1', 0x1), ('CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART1', 1), ('CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART1', 1), ('CFG_MSGOUT_UBX_NAV_PVT_UART1', 1), ('CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1', 1)]) 

        if use_corrections:
            config_data.extend([('CFG_SPARTN_USE_SOURCE', 0), ('CFG_UART1INPROT_SPARTN', 1), ('CFG_MSGOUT_UBX_RXM_SPARTN_UART1', 1), ('CFG_MSGOUT_UBX_RXM_COR_UART1', 1)])
            
        return config_data

    def __save_boot_times(self, time_in_sec: int):
        with open(self.__rtk_mode + '_boot_times.txt', 'a') as log_file:
            log_file.write("\nAntenna Rebooted after sec: " + str(time_in_sec))