#!/usr/bin/env python

from .scservo_def import *

TXPACKET_MAX_LEN = 250
RXPACKET_MAX_LEN = 250

# for Protocol Packet
PKT_HEADER0 = 0
PKT_HEADER1 = 1
PKT_ID = 2
PKT_LENGTH = 3
PKT_INSTRUCTION = 4
PKT_ERROR = 4
PKT_PARAMETER0 = 5

# Protocol Error bit
ERRBIT_VOLTAGE = 1
ERRBIT_ANGLE = 2
ERRBIT_OVERHEAT = 4
ERRBIT_OVERELE = 8
ERRBIT_OVERLOAD = 32


class protocol_packet_handler(object):
<<<<<<< HEAD
    def __init__(self, portHandler, protocol_end):
        #self.scs_setend(protocol_end)# SCServo bit end(STS/SMS=0, SCS=1)
        self.portHandler = portHandler
        self.scs_end = protocol_end

    def scs_getend(self):
        return self.scs_end

    def scs_setend(self, e):
        self.scs_end = e

    def scs_tohost(self, a, b):
        if (a & (1<<b)):
            return -(a & ~(1<<b))
        else:
            return a

    def scs_toscs(self, a, b):
        if (a<0):
            return (-a | (1<<b))
        else:
            return a

    def scs_makeword(self, a, b):
        if self.scs_end==0:
            return (a & 0xFF) | ((b & 0xFF) << 8)
        else:
            return (b & 0xFF) | ((a & 0xFF) << 8)

    def scs_makedword(self, a, b):
        return (a & 0xFFFF) | (b & 0xFFFF) << 16

    def scs_loword(self, l):
        return l & 0xFFFF

    def scs_hiword(self, h):
        return (h >> 16) & 0xFFFF

    def scs_lobyte(self, w):
        if self.scs_end==0:
            return w & 0xFF
        else:
            return (w >> 8) & 0xFF

    def scs_hibyte(self, w):
        if self.scs_end==0:
            return (w >> 8) & 0xFF
        else:
            return w & 0xFF
        
=======
>>>>>>> 78103c8 (ling zu shidai)
    def getProtocolVersion(self):
        return 1.0

    def getTxRxResult(self, result):
        if result == COMM_SUCCESS:
            return "[TxRxResult] Communication success!"
        elif result == COMM_PORT_BUSY:
            return "[TxRxResult] Port is in use!"
        elif result == COMM_TX_FAIL:
            return "[TxRxResult] Failed transmit instruction packet!"
        elif result == COMM_RX_FAIL:
            return "[TxRxResult] Failed get status packet from device!"
        elif result == COMM_TX_ERROR:
            return "[TxRxResult] Incorrect instruction packet!"
        elif result == COMM_RX_WAITING:
            return "[TxRxResult] Now receiving status packet!"
        elif result == COMM_RX_TIMEOUT:
            return "[TxRxResult] There is no status packet!"
        elif result == COMM_RX_CORRUPT:
            return "[TxRxResult] Incorrect status packet!"
        elif result == COMM_NOT_AVAILABLE:
            return "[TxRxResult] Protocol does not support this function!"
        else:
            return ""

    def getRxPacketError(self, error):
        if error & ERRBIT_VOLTAGE:
<<<<<<< HEAD
            return "[ServoStatus] Input voltage error!"

        if error & ERRBIT_ANGLE:
            return "[ServoStatus] Angle sen error!"

        if error & ERRBIT_OVERHEAT:
            return "[ServoStatus] Overheat error!"

        if error & ERRBIT_OVERELE:
            return "[ServoStatus] OverEle error!"
        
        if error & ERRBIT_OVERLOAD:
            return "[ServoStatus] Overload error!"

        return ""

    def txPacket(self, txpacket):
        checksum = 0
        total_packet_length = txpacket[PKT_LENGTH] + 4  # 4: HEADER0 HEADER1 ID LENGTH

        if self.portHandler.is_using:
            return COMM_PORT_BUSY
        self.portHandler.is_using = True

        # check max packet length
        if total_packet_length > TXPACKET_MAX_LEN:
            self.portHandler.is_using = False
=======
            return "[RxPacketError] Input voltage error!"

        if error & ERRBIT_ANGLE:
            return "[RxPacketError] Angle sen error!"

        if error & ERRBIT_OVERHEAT:
            return "[RxPacketError] Overheat error!"

        if error & ERRBIT_OVERELE:
            return "[RxPacketError] OverEle error!"
        
        if error & ERRBIT_OVERLOAD:
            return "[RxPacketError] Overload error!"

        return ""

    def txPacket(self, port, txpacket):
        checksum = 0
        total_packet_length = txpacket[PKT_LENGTH] + 4  # 4: HEADER0 HEADER1 ID LENGTH

        if port.is_using:
            return COMM_PORT_BUSY
        port.is_using = True

        # check max packet length
        if total_packet_length > TXPACKET_MAX_LEN:
            port.is_using = False
>>>>>>> 78103c8 (ling zu shidai)
            return COMM_TX_ERROR

        # make packet header
        txpacket[PKT_HEADER0] = 0xFF
        txpacket[PKT_HEADER1] = 0xFF

        # add a checksum to the packet
        for idx in range(2, total_packet_length - 1):  # except header, checksum
            checksum += txpacket[idx]

        txpacket[total_packet_length - 1] = ~checksum & 0xFF

        #print "[TxPacket] %r" % txpacket

        # tx packet
<<<<<<< HEAD
        self.portHandler.clearPort()
        written_packet_length = self.portHandler.writePort(txpacket)
        if total_packet_length != written_packet_length:
            self.portHandler.is_using = False
=======
        port.clearPort()
        written_packet_length = port.writePort(txpacket)
        if total_packet_length != written_packet_length:
            port.is_using = False
>>>>>>> 78103c8 (ling zu shidai)
            return COMM_TX_FAIL

        return COMM_SUCCESS

<<<<<<< HEAD
    def rxPacket(self):
=======
    def rxPacket(self, port):
>>>>>>> 78103c8 (ling zu shidai)
        rxpacket = []

        result = COMM_TX_FAIL
        checksum = 0
        rx_length = 0
        wait_length = 6  # minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)

        while True:
<<<<<<< HEAD
            rxpacket.extend(self.portHandler.readPort(wait_length - rx_length))
=======
            rxpacket.extend(port.readPort(wait_length - rx_length))
>>>>>>> 78103c8 (ling zu shidai)
            rx_length = len(rxpacket)
            if rx_length >= wait_length:
                # find packet header
                for idx in range(0, (rx_length - 1)):
                    if (rxpacket[idx] == 0xFF) and (rxpacket[idx + 1] == 0xFF):
                        break

                if idx == 0:  # found at the beginning of the packet
                    if (rxpacket[PKT_ID] > 0xFD) or (rxpacket[PKT_LENGTH] > RXPACKET_MAX_LEN) or (
                            rxpacket[PKT_ERROR] > 0x7F):
                        # unavailable ID or unavailable Length or unavailable Error
                        # remove the first byte in the packet
                        del rxpacket[0]
                        rx_length -= 1
                        continue

                    # re-calculate the exact length of the rx packet
                    if wait_length != (rxpacket[PKT_LENGTH] + PKT_LENGTH + 1):
                        wait_length = rxpacket[PKT_LENGTH] + PKT_LENGTH + 1
                        continue

                    if rx_length < wait_length:
                        # check timeout
<<<<<<< HEAD
                        if self.portHandler.isPacketTimeout():
=======
                        if port.isPacketTimeout():
>>>>>>> 78103c8 (ling zu shidai)
                            if rx_length == 0:
                                result = COMM_RX_TIMEOUT
                            else:
                                result = COMM_RX_CORRUPT
                            break
                        else:
                            continue

                    # calculate checksum
                    for i in range(2, wait_length - 1):  # except header, checksum
                        checksum += rxpacket[i]
                    checksum = ~checksum & 0xFF

                    # verify checksum
                    if rxpacket[wait_length - 1] == checksum:
                        result = COMM_SUCCESS
                    else:
                        result = COMM_RX_CORRUPT
                    break

                else:
                    # remove unnecessary packets
                    del rxpacket[0: idx]
                    rx_length -= idx

            else:
                # check timeout
<<<<<<< HEAD
                if self.portHandler.isPacketTimeout():
=======
                if port.isPacketTimeout():
>>>>>>> 78103c8 (ling zu shidai)
                    if rx_length == 0:
                        result = COMM_RX_TIMEOUT
                    else:
                        result = COMM_RX_CORRUPT
                    break

<<<<<<< HEAD
        self.portHandler.is_using = False
        return rxpacket, result

    def txRxPacket(self, txpacket):
=======
        port.is_using = False

        #print "[RxPacket] %r" % rxpacket

        return rxpacket, result

    def txRxPacket(self, port, txpacket):
>>>>>>> 78103c8 (ling zu shidai)
        rxpacket = None
        error = 0

        # tx packet
<<<<<<< HEAD
        result = self.txPacket(txpacket)
=======
        result = self.txPacket(port, txpacket)
>>>>>>> 78103c8 (ling zu shidai)
        if result != COMM_SUCCESS:
            return rxpacket, result, error

        # (ID == Broadcast ID) == no need to wait for status packet or not available
        if (txpacket[PKT_ID] == BROADCAST_ID):
<<<<<<< HEAD
            self.portHandler.is_using = False
=======
            port.is_using = False
>>>>>>> 78103c8 (ling zu shidai)
            return rxpacket, result, error

        # set packet timeout
        if txpacket[PKT_INSTRUCTION] == INST_READ:
<<<<<<< HEAD
            self.portHandler.setPacketTimeout(txpacket[PKT_PARAMETER0 + 1] + 6)
        else:
            self.portHandler.setPacketTimeout(6)  # HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM

        # rx packet
        while True:
            rxpacket, result = self.rxPacket()
=======
            port.setPacketTimeout(txpacket[PKT_PARAMETER0 + 1] + 6)
        else:
            port.setPacketTimeout(6)  # HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM

        # rx packet
        while True:
            rxpacket, result = self.rxPacket(port)
>>>>>>> 78103c8 (ling zu shidai)
            if result != COMM_SUCCESS or txpacket[PKT_ID] == rxpacket[PKT_ID]:
                break

        if result == COMM_SUCCESS and txpacket[PKT_ID] == rxpacket[PKT_ID]:
            error = rxpacket[PKT_ERROR]

        return rxpacket, result, error

<<<<<<< HEAD
    def ping(self, scs_id):
=======
    def ping(self, port, scs_id):
>>>>>>> 78103c8 (ling zu shidai)
        model_number = 0
        error = 0

        txpacket = [0] * 6

<<<<<<< HEAD
        if scs_id > BROADCAST_ID:
=======
        if scs_id >= BROADCAST_ID:
>>>>>>> 78103c8 (ling zu shidai)
            return model_number, COMM_NOT_AVAILABLE, error

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = 2
        txpacket[PKT_INSTRUCTION] = INST_PING

<<<<<<< HEAD
        rxpacket, result, error = self.txRxPacket(txpacket)

        if result == COMM_SUCCESS:
            data_read, result, error = self.readTxRx(scs_id, 3, 2)  # Address 3 : Model Number
            if result == COMM_SUCCESS:
                model_number = self.scs_makeword(data_read[0], data_read[1])

        return model_number, result, error

    def action(self, scs_id):
=======
        rxpacket, result, error = self.txRxPacket(port, txpacket)

        if result == COMM_SUCCESS:
            data_read, result, error = self.readTxRx(port, scs_id, 3, 2)  # Address 3 : Model Number
            if result == COMM_SUCCESS:
                model_number = SCS_MAKEWORD(data_read[0], data_read[1])

        return model_number, result, error

    def action(self, port, scs_id):
>>>>>>> 78103c8 (ling zu shidai)
        txpacket = [0] * 6

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = 2
        txpacket[PKT_INSTRUCTION] = INST_ACTION

<<<<<<< HEAD
        _, result, _ = self.txRxPacket(txpacket)

        return result

    def readTx(self, scs_id, address, length):

        txpacket = [0] * 8

        if scs_id > BROADCAST_ID:
=======
        _, result, _ = self.txRxPacket(port, txpacket)

        return result

    def readTx(self, port, scs_id, address, length):

        txpacket = [0] * 8

        if scs_id >= BROADCAST_ID:
>>>>>>> 78103c8 (ling zu shidai)
            return COMM_NOT_AVAILABLE

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = 4
        txpacket[PKT_INSTRUCTION] = INST_READ
        txpacket[PKT_PARAMETER0 + 0] = address
        txpacket[PKT_PARAMETER0 + 1] = length

<<<<<<< HEAD
        result = self.txPacket(txpacket)

        # set packet timeout
        if result == COMM_SUCCESS:
            self.portHandler.setPacketTimeout(length + 6)

        return result

    def readRx(self, scs_id, length):
=======
        result = self.txPacket(port, txpacket)

        # set packet timeout
        if result == COMM_SUCCESS:
            port.setPacketTimeout(length + 6)

        return result

    def readRx(self, port, scs_id, length):
>>>>>>> 78103c8 (ling zu shidai)
        result = COMM_TX_FAIL
        error = 0

        rxpacket = None
        data = []

        while True:
<<<<<<< HEAD
            rxpacket, result = self.rxPacket()
=======
            rxpacket, result = self.rxPacket(port)
>>>>>>> 78103c8 (ling zu shidai)

            if result != COMM_SUCCESS or rxpacket[PKT_ID] == scs_id:
                break

        if result == COMM_SUCCESS and rxpacket[PKT_ID] == scs_id:
            error = rxpacket[PKT_ERROR]

<<<<<<< HEAD
            data.extend(rxpacket[PKT_PARAMETER0 : PKT_PARAMETER0+length])

        return data, result, error

    def readTxRx(self, scs_id, address, length):
        txpacket = [0] * 8
        data = []

        if scs_id > BROADCAST_ID:
=======
            data.extend(rxpacket[PKT_PARAMETER0: PKT_PARAMETER0 + length])

        return data, result, error

    def readTxRx(self, port, scs_id, address, length):
        txpacket = [0] * 8
        data = []

        if scs_id >= BROADCAST_ID:
>>>>>>> 78103c8 (ling zu shidai)
            return data, COMM_NOT_AVAILABLE, 0

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = 4
        txpacket[PKT_INSTRUCTION] = INST_READ
        txpacket[PKT_PARAMETER0 + 0] = address
        txpacket[PKT_PARAMETER0 + 1] = length

<<<<<<< HEAD
        rxpacket, result, error = self.txRxPacket(txpacket)
        if result == COMM_SUCCESS:
            error = rxpacket[PKT_ERROR]

            data.extend(rxpacket[PKT_PARAMETER0 : PKT_PARAMETER0+length])

        return data, result, error

    def read1ByteTx(self, scs_id, address):
        return self.readTx(scs_id, address, 1)

    def read1ByteRx(self, scs_id):
        data, result, error = self.readRx(scs_id, 1)
        data_read = data[0] if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read1ByteTxRx(self, scs_id, address):
        data, result, error = self.readTxRx(scs_id, address, 1)
        data_read = data[0] if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read2ByteTx(self, scs_id, address):
        return self.readTx(scs_id, address, 2)

    def read2ByteRx(self, scs_id):
        data, result, error = self.readRx(scs_id, 2)
        data_read = self.scs_makeword(data[0], data[1]) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read2ByteTxRx(self, scs_id, address):
        data, result, error = self.readTxRx(scs_id, address, 2)
        data_read = self.scs_makeword(data[0], data[1]) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read4ByteTx(self, scs_id, address):
        return self.readTx(scs_id, address, 4)

    def read4ByteRx(self, scs_id):
        data, result, error = self.readRx(scs_id, 4)
        data_read = self.scs_makedword(self.scs_makeword(data[0], data[1]),
                                  self.scs_makeword(data[2], data[3])) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read4ByteTxRx(self, scs_id, address):
        data, result, error = self.readTxRx(scs_id, address, 4)
        data_read = self.scs_makedword(self.scs_makeword(data[0], data[1]),
                                  self.scs_makeword(data[2], data[3])) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def writeTxOnly(self, scs_id, address, length, data):
=======
        rxpacket, result, error = self.txRxPacket(port, txpacket)
        if result == COMM_SUCCESS:
            error = rxpacket[PKT_ERROR]

            data.extend(rxpacket[PKT_PARAMETER0: PKT_PARAMETER0 + length])

        return data, result, error

    def read1ByteTx(self, port, scs_id, address):
        return self.readTx(port, scs_id, address, 1)

    def read1ByteRx(self, port, scs_id):
        data, result, error = self.readRx(port, scs_id, 1)
        data_read = data[0] if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read1ByteTxRx(self, port, scs_id, address):
        data, result, error = self.readTxRx(port, scs_id, address, 1)
        data_read = data[0] if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read2ByteTx(self, port, scs_id, address):
        return self.readTx(port, scs_id, address, 2)

    def read2ByteRx(self, port, scs_id):
        data, result, error = self.readRx(port, scs_id, 2)
        data_read = SCS_MAKEWORD(data[0], data[1]) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read2ByteTxRx(self, port, scs_id, address):
        data, result, error = self.readTxRx(port, scs_id, address, 2)
        data_read = SCS_MAKEWORD(data[0], data[1]) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read4ByteTx(self, port, scs_id, address):
        return self.readTx(port, scs_id, address, 4)

    def read4ByteRx(self, port, scs_id):
        data, result, error = self.readRx(port, scs_id, 4)
        data_read = SCS_MAKEDWORD(SCS_MAKEWORD(data[0], data[1]),
                                  SCS_MAKEWORD(data[2], data[3])) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def read4ByteTxRx(self, port, scs_id, address):
        data, result, error = self.readTxRx(port, scs_id, address, 4)
        data_read = SCS_MAKEDWORD(SCS_MAKEWORD(data[0], data[1]),
                                  SCS_MAKEWORD(data[2], data[3])) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def writeTxOnly(self, port, scs_id, address, length, data):
>>>>>>> 78103c8 (ling zu shidai)
        txpacket = [0] * (length + 7)

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = length + 3
        txpacket[PKT_INSTRUCTION] = INST_WRITE
        txpacket[PKT_PARAMETER0] = address

        txpacket[PKT_PARAMETER0 + 1: PKT_PARAMETER0 + 1 + length] = data[0: length]

<<<<<<< HEAD
        result = self.txPacket(txpacket)
        self.portHandler.is_using = False

        return result

    def writeTxRx(self, scs_id, address, length, data):
=======
        result = self.txPacket(port, txpacket)
        port.is_using = False

        return result

    def writeTxRx(self, port, scs_id, address, length, data):
>>>>>>> 78103c8 (ling zu shidai)
        txpacket = [0] * (length + 7)

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = length + 3
        txpacket[PKT_INSTRUCTION] = INST_WRITE
        txpacket[PKT_PARAMETER0] = address

        txpacket[PKT_PARAMETER0 + 1: PKT_PARAMETER0 + 1 + length] = data[0: length]
<<<<<<< HEAD
        rxpacket, result, error = self.txRxPacket(txpacket)

        return result, error

    def write1ByteTxOnly(self, scs_id, address, data):
        data_write = [data]
        return self.writeTxOnly(scs_id, address, 1, data_write)

    def write1ByteTxRx(self, scs_id, address, data):
        data_write = [data]
        return self.writeTxRx(scs_id, address, 1, data_write)

    def write2ByteTxOnly(self, scs_id, address, data):
        data_write = [self.scs_lobyte(data), self.scs_hibyte(data)]
        return self.writeTxOnly(scs_id, address, 2, data_write)

    def write2ByteTxRx(self, scs_id, address, data):
        data_write = [self.scs_lobyte(data), self.scs_hibyte(data)]
        return self.writeTxRx(scs_id, address, 2, data_write)

    def write4ByteTxOnly(self, scs_id, address, data):
        data_write = [self.scs_lobyte(self.scs_loword(data)),
                      self.scs_hibyte(self.scs_loword(data)),
                      self.scs_lobyte(self.scs_hiword(data)),
                      self.scs_hibyte(self.scs_hiword(data))]
        return self.writeTxOnly(scs_id, address, 4, data_write)

    def write4ByteTxRx(self, scs_id, address, data):
        data_write = [self.scs_lobyte(self.scs_loword(data)),
                      self.scs_hibyte(self.scs_loword(data)),
                      self.scs_lobyte(self.scs_hiword(data)),
                      self.scs_hibyte(self.scs_hiword(data))]
        return self.writeTxRx(scs_id, address, 4, data_write)

    def regWriteTxOnly(self, scs_id, address, length, data):
=======
        rxpacket, result, error = self.txRxPacket(port, txpacket)

        return result, error

    def write1ByteTxOnly(self, port, scs_id, address, data):
        data_write = [data]
        return self.writeTxOnly(port, scs_id, address, 1, data_write)

    def write1ByteTxRx(self, port, scs_id, address, data):
        data_write = [data]
        return self.writeTxRx(port, scs_id, address, 1, data_write)

    def write2ByteTxOnly(self, port, scs_id, address, data):
        data_write = [SCS_LOBYTE(data), SCS_HIBYTE(data)]
        return self.writeTxOnly(port, scs_id, address, 2, data_write)

    def write2ByteTxRx(self, port, scs_id, address, data):
        data_write = [SCS_LOBYTE(data), SCS_HIBYTE(data)]
        return self.writeTxRx(port, scs_id, address, 2, data_write)

    def write4ByteTxOnly(self, port, scs_id, address, data):
        data_write = [SCS_LOBYTE(SCS_LOWORD(data)),
                      SCS_HIBYTE(SCS_LOWORD(data)),
                      SCS_LOBYTE(SCS_HIWORD(data)),
                      SCS_HIBYTE(SCS_HIWORD(data))]
        return self.writeTxOnly(port, scs_id, address, 4, data_write)

    def write4ByteTxRx(self, port, scs_id, address, data):
        data_write = [SCS_LOBYTE(SCS_LOWORD(data)),
                      SCS_HIBYTE(SCS_LOWORD(data)),
                      SCS_LOBYTE(SCS_HIWORD(data)),
                      SCS_HIBYTE(SCS_HIWORD(data))]
        return self.writeTxRx(port, scs_id, address, 4, data_write)

    def regWriteTxOnly(self, port, scs_id, address, length, data):
>>>>>>> 78103c8 (ling zu shidai)
        txpacket = [0] * (length + 7)

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = length + 3
        txpacket[PKT_INSTRUCTION] = INST_REG_WRITE
        txpacket[PKT_PARAMETER0] = address

        txpacket[PKT_PARAMETER0 + 1: PKT_PARAMETER0 + 1 + length] = data[0: length]

<<<<<<< HEAD
        result = self.txPacket(txpacket)
        self.portHandler.is_using = False

        return result

    def regWriteTxRx(self, scs_id, address, length, data):
=======
        result = self.txPacket(port, txpacket)
        port.is_using = False

        return result

    def regWriteTxRx(self, port, scs_id, address, length, data):
>>>>>>> 78103c8 (ling zu shidai)
        txpacket = [0] * (length + 7)

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = length + 3
        txpacket[PKT_INSTRUCTION] = INST_REG_WRITE
        txpacket[PKT_PARAMETER0] = address

        txpacket[PKT_PARAMETER0 + 1: PKT_PARAMETER0 + 1 + length] = data[0: length]

<<<<<<< HEAD
        _, result, error = self.txRxPacket(txpacket)

        return result, error

    def syncReadTx(self, start_address, data_length, param, param_length):
=======
        _, result, error = self.txRxPacket(port, txpacket)

        return result, error

    def syncReadTx(self, port, start_address, data_length, param, param_length):
>>>>>>> 78103c8 (ling zu shidai)
        txpacket = [0] * (param_length + 8)
        # 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN CHKSUM

        txpacket[PKT_ID] = BROADCAST_ID
        txpacket[PKT_LENGTH] = param_length + 4  # 7: INST START_ADDR DATA_LEN CHKSUM
        txpacket[PKT_INSTRUCTION] = INST_SYNC_READ
        txpacket[PKT_PARAMETER0 + 0] = start_address
        txpacket[PKT_PARAMETER0 + 1] = data_length

        txpacket[PKT_PARAMETER0 + 2: PKT_PARAMETER0 + 2 + param_length] = param[0: param_length]

<<<<<<< HEAD
        # print(txpacket)
        result = self.txPacket(txpacket)
        return result

    def syncReadRx(self, data_length, param_length):
        wait_length = (6 + data_length) * param_length
        self.portHandler.setPacketTimeout(wait_length)
        rxpacket = []
        rx_length = 0
        while True:
            rxpacket.extend(self.portHandler.readPort(wait_length - rx_length))
            rx_length = len(rxpacket)
            if rx_length >= wait_length:
                result = COMM_SUCCESS
                break
            else:
                # check timeout
                if self.portHandler.isPacketTimeout():
                    if rx_length == 0:
                        result = COMM_RX_TIMEOUT
                    else:
                        result = COMM_RX_CORRUPT
                    break
        self.portHandler.is_using = False
        return result, rxpacket

    def syncWriteTxOnly(self, start_address, data_length, param, param_length):
=======
        result = self.txPacket(port, txpacket)
        if result == COMM_SUCCESS:
            port.setPacketTimeout((6 + data_length) * param_length)

        return result


    def syncWriteTxOnly(self, port, start_address, data_length, param, param_length):
>>>>>>> 78103c8 (ling zu shidai)
        txpacket = [0] * (param_length + 8)
        # 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN ... CHKSUM

        txpacket[PKT_ID] = BROADCAST_ID
        txpacket[PKT_LENGTH] = param_length + 4  # 4: INST START_ADDR DATA_LEN ... CHKSUM
        txpacket[PKT_INSTRUCTION] = INST_SYNC_WRITE
        txpacket[PKT_PARAMETER0 + 0] = start_address
        txpacket[PKT_PARAMETER0 + 1] = data_length

        txpacket[PKT_PARAMETER0 + 2: PKT_PARAMETER0 + 2 + param_length] = param[0: param_length]

<<<<<<< HEAD
        _, result, _ = self.txRxPacket(txpacket)

        return result

    
    def reOfsCal(self, scs_id, position):
        error = 0

        txpacket = [0] * 8

        if scs_id > BROADCAST_ID:
            return COMM_NOT_AVAILABLE, error

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = 4
        txpacket[PKT_INSTRUCTION] = INST_OFSCAL
        txpacket[PKT_PARAMETER0] = self.scs_lobyte(position)
        txpacket[PKT_PARAMETER0+1] = self.scs_hibyte(position)

        rxpacket, result, error = self.txRxPacket(txpacket)

        return result, error

    def reSet(self, scs_id):
        error = 0

        txpacket = [0] * 6

        if scs_id > BROADCAST_ID:
            return COMM_NOT_AVAILABLE, error

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = 2
        txpacket[PKT_INSTRUCTION] = INST_RESET

        rxpacket, result, error = self.txRxPacket(txpacket)

        return result, error
=======
        _, result, _ = self.txRxPacket(port, txpacket)

        return result
>>>>>>> 78103c8 (ling zu shidai)
