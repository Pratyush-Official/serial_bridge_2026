import serial
import time
import logging
from myserialbridge.crc8 import crc8

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

START_BYTE = 0xA5

class serial_comms:
    def __init__(self, serial_port, serial_baudrate, rx_data_size,  tx_data_size, hash_func_="CRC"):
        try:
            self.serial = serial.Serial(serial_port, serial_baudrate, timeout = 1.0)
        except serial.SerialException as e:
            logger.error(f"[Serial_comms]:Serial port error : {e}")
            self._reopen_serial_port()

        # self.serial = None
        self.serial_port = serial_port
        self.serial_baudrate = serial_baudrate
        self.rx_data_size = rx_data_size
        self.tx_data_size = tx_data_size
        self.hash_func = hash_func_
        self.hash_not_matched_count = 0
        self.start_byte_found = False

    def write_data(self, data):
        try:
            self.serial.write(data)
        except serial.SerialException as e:
            logger.error(f"Error as : {e}")

    def read_data(self):

        # if self.serial is None:
        #     return None

        if not self.start_byte_found:

            try:
                if self.serial.in_waiting >= 1:
                    byte = self.serial.read(1)
                    if int.from_bytes(byte, 'big') == START_BYTE:
                        self.start_byte_found =True
                        # print("FOUND", int.from_bytes(byte, 'big'))

                    else:
                        print(int.from_bytes(byte, 'big'))
                        print("[Serial_coms]: Start byte not matched")
                else:
                    # print("[Serial_coms]:timeoutwaiting for start byte")
                    return
            except(serial.SerialException, OSError) as e:
                logger.error(f"[Serial_coms]::Serial port error: {e}")
                self._reopen_serial_port()

        else:
            try:
                if self.serial.in_waiting >= self.rx_data_size - 1:
                    self.start_byte_found = False
                    data_str = self.serial.read(self.rx_data_size -1)
                    
                    hash = self.calc_crc(data_str[0:-1])
                    if hash == data_str[-1]:
                        self.hash_not_matched_count = 0
                        return data_str
                    else:
                        self.hash_not_matched_count += 1
                        print(f"[Serial_coms]: hash not matched, count: {self.hash_not_matched_count}")
                        return
            except (serial.SerialException, OSError) as e:
                logger.error(f"[Serial_coms]:Serial port error: {e}")
                self._reopen_serial_port()

    def calc_crc(self, data=[]):
      
        hash_func=crc8()
        hash_func.update(data)
        return hash_func.digest()[0]


    def _reopen_serial_port(self):
        try:
            time.sleep(2)
            self.serial.close()
            self.serial = serial.Serial(self.serial_port,self.serial_baudrate, timeout=1.0)
            logger.info("[Serial_comms]: Serial port reopened successfully")
        except Exception as e:
            logger.error(f"[Serial_comms]: Failed to reopen serial port: {e}")

    def close(self):
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
            logger.info("Port closed")



    



