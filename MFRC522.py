#!/usr/bin/env python3.5

import logging
import os
import signal
import spidev
import sys
import time
import RPi.GPIO as GPIO

##              FUNCTION LIST               ##
#
#   MFRC522 LIBRARY
#
#   1.  __init__(self, bus=0, device=0, spd=1000000, pin_mode=10, pin_RESET=-1, debugLevel='WARNING')
#   2.  MFRC522_Reset(self)
#   3.  Write_MFRC522(self, addr, val)
#   4.  Read_MFRC522(self, addr)
#   5.  Close_MFRC522(self)
#   6.  SetBitMask(self, reg, mask)
#   7.  ClearBitMask(self, reg, mask)
#   8.  AntennaOn(self)
#   9.  AntennaOff(self)
#   10. MFRC522_ToCard(self, command, sendData)
#   11. MFRC522_Request(self, reqMode)
#   12. MFRC522_Anticoll(self)
#   13. CalculateCRC(self, pIndata)
#   14. MFRC522_SelectTag(self, serNum)
#   15. MFRC522_Auth(self, authMode, BlockAddr, Sectorkey, serNum)
#   16. MFRC522_StopCrypto1(self)
#   17. MFRC522_Read(self, blockAddr)
#   18. MFRC522_Write(self, blockAddr, writeData)
#   19. MFRC522_DumpClassic1K(self, key, uid)
#   20. MFRC522_Init(self)
#
#
#   Encryption Program Related Functions
#
#   1.  signal_handler(sig, frame)
#   2.  scan_tag()
#   3.  get_ID(tag)
#   4.  get_data(tag)
#   5.  check_tag(scanned_id, scanned_data)
#   6.  add_tag(tag_list)
#   7.  get_num_tags()
#   8.  delete_file(fName)
#   9.  display_file(fName)
#   10. write_to_records()
#   11. overwrite_records()
#   12. replace_tag()
#   13. remove_tag()
#   14. detect_tag(search_ID)



    ##	COMMAND AND STATUS REGISTERS	##
    #   VARIABLE                #   BINARY      #   FUNCTIONS USED IN                                           #   DESCRIPTION     #
    #
    Reserved00 = 0x00	        #	0000 0000	#   NOT USED                                                    #   Reserved for future use
    CommandReg = 0x01	        #	0000 0001	#   MFRC522_Reset(), MFRC522_ToCard(), CalculateCRC()           #   Starts/Stops command execution                                    
    CommIEnReg = 0x02	        #	0000 0010	#   MFRC522_ToCard()                                            #   Enable/Disable interrupt request control bits
    DivlEnReg = 0x03	        #	0000 0011	#   NOT USED                                                    #   Enable/Disable interrupt request control bits
    CommIrqReg = 0x04	        #	0000 0100	#   MFRC522_ToCard()                                            #   Interrupt request bits
    DivIrqReg = 0x05	        #	0000 0101	#   CalculateCRC()                                              #   Interrupt request bits
    ErrorReg = 0x06		        #	0000 0110	#   MFRC522_ToCard()                                            #   Error bits showing the error status of last command executed
    StatusReg = 0x07	        #	0000 0111	#   NOT USED                                                    #   Communication status bits
    Status2Reg = 0x08	        #	0000 1000	#   MFRC522_Auth(), MFRC522_StopCrypto1()                       #   Receiver/Transmitter status bits
    FIFODataReg = 0x09	        #	0000 1001	#   MFRC522_ToCard(), CalculateCRC()                            #   I/O of 64-byte FIFO buffer
    FIFOLevelReg = 0x0A	        #  	0000 1010	#   MFRC522_ToCard(), CalculateCRC()                            #   Number of bytes stored in FIFO buffer
    WaterLevelReg = 0x0B	    #	0000 1011	#   NOT USED                                                    #   Level for FIFO underflow and overflow warning
    ControlReg = 0x0C	        #	0000 1100	#   MFRC522_ToCard()                                            #   Miscellaneous control registers
    BitFramingReg = 0x0D	    #	0000 1101	#   MFRC522_ToCard(), MFRC522_Request(), MFRC522_Anticoll()     #   Adjustments for bit-oriented frames
    CollReg = 0x0E		        #	0000 1110	#   NOT USED                                                    #   Bit position of the bfirst bit-collision detected on the RF interface
    Reserved0F = 0x0F	        #	0000 1111	#   NOT USED                                                    #   Reserved

    ##		COMMAND			    ##
    #   VARIABLE                #   BINARY      #   FUNCTION USED IN                                            #   DESCRIPTION     #
    #
    Reserved10 = 0x10 	        #	0001 0000	#   NOT USED                                                    #   Reserved
    ModeReg = 0x11		        #	0001 0001	#   MFRC522_Init()                                              #   Defines general modes for transmitting and receiving
    TxModeReg = 0x12	        #	0001 0010	#   NOT USED                                                    #   Defines transmission data rate and framing
    RxModeReg = 0x13	        #	0001 0011	#   NOT USED                                                    #   Defines reception data rate and framing
    TxControlReg = 0x14	        #	0001 0100	#   AntennaOn(), AntennaOff()                                   #   Controls logical behavior of the antenna driver pins TX1 and TX2
    TxASKReg = 0x15		        #	0001 0101	#   MFRC522_Init()                                              #   Controls the setting of the transmission modulation
    TxSelReg = 0x16		        #	0001 0110	#   NOT USED                                                    #   Selects the internal sources for the antenna driver
    RxSelReg = 0x17		        #	0001 0111	#   NOT USED                                                    #   Selects internal receiver settings
    RxThresholdReg = 0x18	    #	0001 1000	#   NOT USED                                                    #   Selects thresholds for the bit decoder
    DemodReg = 0x19		        #	0001 1001	#   NOT USED                                                    #   Defines demodulator settings
    Reserved1A = 0x1A	        #	0001 1010	#   NOT USED                                                    #   Reserved
    Reserved1B = 0x1B	        #	0001 1011	#   NOT USED                                                    #   Reserved
    MfTxReg = 0x1C		        #	0001 1100	#   NOT USED                                                    #   Controls some MIFARE communication transmit parameters
    Reserved1D = 0x1D	        #	0001 1101	#   NOT USED                                                    #   Reserved
    Reserved1E = 0x1E	        #	0001 1110	#   NOT USED                                                    #   Reserved
    SerialSpeedReg = 0x1F	    #	0001 1111	#   NOT USED                                                    #   Selects speed of the serial UART interface

    ##    	CONFIGURATION		##
    #   VARIABLE                #   BINARY      #   FUNCTION USED IN                                            #   DESCRIPTION     #
    #
    Reserved20 = 0x20	        #	0010 0000	#   NOT USED                                                    #   Reserved
    CRCResultRegM = 0x21	    #	0010 0001	#   CalculateCRC()                                              #   MSB Values of CRC calculation
    CRCResultRegL = 0x22	    #	0010 0010	#   CalculateCRC()                                              #   LSB Values of CRC calculation
    Reserved23 = 0x23	        #	0010 0011	#   NOT USED                                                    #   Reserved
    ModWidthReg = 0x24	        #	0010 0100	#   NOT USED                                                    #   Controls ModWidth setting
    Reserved25 = 0x25	        #	0010 0101	#   NOT USED                                                    #   Reserved
    RFCfgReg = 0x26		        #  	0010 0110	#   NOT USED                                                    #   Configure Receiver GAIN
    GsNReg = 0x27		        #	0010 0111	#   NOT USED                                                    #   Selects conductance of the antenna driver pins TX1 and TX2 for modulation
    CWGsPReg = 0x28		        #	0010 1000	#   NOT USED                                                    #   Defines conductance of the p-driver output during periods of <<NO>> modulation
    ModGsPReg = 0x29	        #	0010 1001	#   NOT USED                                                    #   Defines conductance of the p-driver output <<DURING>> periods of modulation
    TModeReg = 0x2A		        #	0010 1010	#   MFRC522_Init()                                              #   Defines settings for internal timer
    TPrescalerReg = 0x2B	    #	0010 1011	#   MFRC522_Init()                                              #   Defines settings for internal timer
    TReloadRegH = 0x2C	        #	0010 1100	#   MFRC522_Init()                                              #   MSB of 16-bit Timer reload value
    TReloadRegL = 0x2D	        #	0010 1101	#   MFRC522_Init()                                              #   LSB of 16-bit Timer reload value
    TCounterValueRegH = 0x2E    #	0010 1110	#   NOT USED                                                    #   MSB of 16-bit Timer value
    TCounterValueRegL = 0x2F    #	0010 1111	#   NOT USED                                                    #   LSB of 16-bit Timer value

    ##		TEST REGISTER		##
    #   VARIABLE                #   BINARY      #   FUNCTION USED IN                                            #   DESCRIPTION     #
    #
    Reserved30 = 0x30	        #	0011 0000	#   NOT USED                                                    #   Reserved
    TestSel1Reg = 0x31	        #	0011 0001	#   NOT USED                                                    #   General test signal configuration
    TestSel2Reg = 0x32	        #	0011 0010	#   NOT USED                                                    #   General test signal configuration AND PRBS control
    TestPinEnReg = 0x33	        #	0011 0011	#   NOT USED                                                    #   Enables pin output driver on pins D1 to D7
    TestPinValueReg = 0x34	    #	0011 0100	#   NOT USED                                                    #   Defines values for D1 to D7 when it is used as an I/O bus
    TestBusReg = 0x35	        #	0011 0101	#   NOT USED                                                    #   Shows the status of the internal test bus
    AutoTestReg = 0x36	        #	0011 0110	#   NOT USED                                                    #   Controls the digital self test
    VersionReg = 0x37	        #	0011 0111	#   NOT USED                                                    #   Shows the software version
    AnalogTestReg = 0x38	    #	0011 1000	#   NOT USED                                                    #   Controls the pins AUX1 and AUX2
    TestDAC1Reg = 0x39	        #	0011 1001	#   NOT USED                                                    #   Defines test values for TestDAC1
    TestDAC2Reg = 0x3A	        #	0011 1010	#   NOT USED                                                    #   Defines test values for TestDAC2
    TestADCReg = 0x3B	        #	0011 1011	#   NOT USED                                                    #   Shows the value of ADC I and Q channels
    Reserved3C = 0x3C	        #	0011 1100	#   NOT USED                                                    #   Reserved for production tests
    Reserved3D = 0x3D	        #	0011 1101	#   NOT USED
    Reserved3E = 0x3E	        #	0011 1110	#   NOT USED
    Reserved3F = 0x3F	        #	0011 1111	#   NOT USED



    ##      MFRC22 COMMAND WORDS        ##
    #   VARIABLE                #   BINARY      #   FUNCTION USED IN                                            #   DESCRIPTION     #
    #
    PCD_IDLE = 0x00		        #	0000 0000   #   MFRC522_ToCard()                                            #   NO Action. Cancel current command
    PCD_AUTHENT = 0x0E	        #	0000 1110   #   MFRC522_ToCard() and MFRC522_Auth()                         #   Verify Key
    PCD_RECEIVE = 0x08	        #	0000 1000   #   NOT USED                                                    #   Receive Data
    PCD_TRANSMIT = 0x04	        #	0000 0100   #   NOT USED                                                    #   Transmit Data

    PCD_TRANSCEIVE = 0x0C	    #	0000 1100   #   MFRC522_ToCard(), MFRC522_Request(), MFRC522_Anticoll(),    #   Receive AND Send Data
                                                #   MFRC522_SelectTag(), MFRC522_Read(), MFRC522_Write()

    PCD_RESETPHASE = 0x0F	    #	0000 1111   #   MFRC522_Reset()                                             #   Reset
    PCD_CALCCRC = 0x03	        #	0000 0011   #   CalculateCRC()                                              #   CRC Calculation



    ##      MIFARE_ONE CARD COMMAND WORDS       ##
    #   VARIABLE                #   BINARY      #   FUNCTION USED IN                                            #   DESCRIPTION     #
    #
    PICC_REQIDL = 0x26	        #	0010 0110   #   CALLED FROM SimpleMFRC522                                   #   Line-Tracking area is DORMANT
    PICC_REQALL = 0x52	        #	0101 0010   #   NOT USED                                                    #   Line-Tracking area is INTERFERED
    PICC_ANTICOLL = 0x93	    #	1001 0011   #   MFRC522_Anticoll()                                          #   Anti-Collision
    PICC_SELECTTAG = 0x93	    #	1001 0011   #   MFRC522_SelectTag()                                         #   Choose Cards
    PICC_AUTHENT1A = 0x60	    #	0110 0000   #   MFRC522_DumpClassic1K()                                     #   VERIFY A Key
    PICC_AUTHENT1B = 0x61	    #	0110 0001   #   NOT USED                                                    #   VERIFY B Key
    PICC_READ = 0x30	        #	0011 0000   #   MFRC522_Read()                                              #   Reader Module
    PICC_WRITE = 0xA0	        #	1010 0000   #   MFRC522_Write()                                             #   Letter block
    PICC_DECREMENT = 0xC0	    #	1100 0000   #   NOT USED
    PICC_INCREMENT = 0xC1	    #  	1100 0001   #   NOT USED
    PICC_RESTORE = 0xC2	        #	1100 0010   #   NOT USED                                                    #   Transfer data to BUFFER
    PICC_TRANSFER = 0xB0	    #	1011 0000   #   NOT USED                                                    #   Save BUFFER data
    PICC_HALT = 0x50	        #	0101 0000   #   NOT USED                                                    #   Dormant


    MAX_LEN = 16
    MI_OK = 0
    MI_NOTAGERR = 1
    MI_ERR = 2

    serNum = []


    def __init__(self, spd=1000000, debugLevel='WARNING'):
        self.spi = spidev.SpiDev()      #   Assign self to SPI module
        self.spi.open(0, 0)             #   Open SPI
        self.spi.max_speed_hz = spd     #   Set MAX Speed to 1MHz
      

        self.logger = logging.getLogger('mfrc522Logger')
        self.logger.addHandler(logging.StreamHandler())
        level = logging.getLevelName(debugLevel)
        self.logger.setLevel(level)

        pin_RESET = 22    
        GPIO.setup(pin_rst, GPIO.OUT)           # Assign RESET pin to GPIO 22 as OUTPUT
        GPIO.output(pin_rst, 1)                 # Drive RESET HIGH
        self.MFRC522_Init()                     # Initialize MFRC522 Reader



    def MFRC522_Reset(self):
        self.Write_MFRC522(self.CommandReg, self.PCD_RESETPHASE)

    def Write_MFRC522(self, addr, val):
        val = self.spi.xfer2([(addr << 1) & 0x7E, val])

    def Read_MFRC522(self, addr):
        val = self.spi.xfer2([((addr << 1) & 0x7E) | 0x80, 0])
        return val[1]

    def Close_MFRC522(self):
        self.spi.close()        # close SPI port
        GPIO.cleanup()          # clean GPIO

    def SetBitMask(self, reg, mask):
        tmp = self.Read_MFRC522(reg)
        self.Write_MFRC522(reg, tmp | mask)

    def ClearBitMask(self, reg, mask):
        tmp = self.Read_MFRC522(reg)
        self.Write_MFRC522(reg, tmp & (~mask))

    def AntennaOn(self):
        temp = self.Read_MFRC522(self.TxControlReg)
        if (~(temp & 0x03)):
            self.SetBitMask(self.TxControlReg, 0x03)

    def AntennaOff(self):
        self.ClearBitMask(self.TxControlReg, 0x03)

    def MFRC522_ToCard(self, command, sendData):
        backData = []
        backLen = 0
        status = self.MI_ERR
        irqEn = 0x00
        waitIRq = 0x00
        lastBits = None
        n = 0

        if command == self.PCD_AUTHENT:
            irqEn = 0x12
            waitIRq = 0x10
        if command == self.PCD_TRANSCEIVE:
            irqEn = 0x77
            waitIRq = 0x30

        self.Write_MFRC522(self.CommIEnReg, irqEn | 0x80)
        self.ClearBitMask(self.CommIrqReg, 0x80)
        self.SetBitMask(self.FIFOLevelReg, 0x80)

        self.Write_MFRC522(self.CommandReg, self.PCD_IDLE)

        for i in range(len(sendData)):
            self.Write_MFRC522(self.FIFODataReg, sendData[i])

        self.Write_MFRC522(self.CommandReg, command)

        if command == self.PCD_TRANSCEIVE:
            self.SetBitMask(self.BitFramingReg, 0x80)

        i = 2000
        while True:
            n = self.Read_MFRC522(self.CommIrqReg)
            i -= 1
            if ~((i != 0) and ~(n & 0x01) and ~(n & waitIRq)):
                break

        self.ClearBitMask(self.BitFramingReg, 0x80)

        if i != 0:
            if (self.Read_MFRC522(self.ErrorReg) & 0x1B) == 0x00:
                status = self.MI_OK

                if n & irqEn & 0x01:
                    status = self.MI_NOTAGERR

                if command == self.PCD_TRANSCEIVE:
                    n = self.Read_MFRC522(self.FIFOLevelReg)
                    lastBits = self.Read_MFRC522(self.ControlReg) & 0x07
                    if lastBits != 0:
                        backLen = (n - 1) * 8 + lastBits
                    else:
                        backLen = n * 8

                    if n == 0:
                        n = 1
                    if n > self.MAX_LEN:
                        n = self.MAX_LEN

                    for i in range(n):
                        backData.append(self.Read_MFRC522(self.FIFODataReg))
            else:
                status = self.MI_ERR

        return (status, backData, backLen)

    def MFRC522_Request(self, reqMode):
        status = None
        backBits = None
        TagType = []

        self.Write_MFRC522(self.BitFramingReg, 0x07)

        TagType.append(reqMode)
        (status, backData, backBits) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, TagType)

        if ((status != self.MI_OK) | (backBits != 0x10)):
            status = self.MI_ERR

        return (status, backBits)

    def MFRC522_Anticoll(self):
        backData = []
        serNumCheck = 0

        serNum = []

        self.Write_MFRC522(self.BitFramingReg, 0x00)

        serNum.append(self.PICC_ANTICOLL)
        serNum.append(0x20)

        (status, backData, backBits) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, serNum)

        if (status == self.MI_OK):
            i = 0
            if len(backData) == 5:
                for i in range(4):
                    serNumCheck = serNumCheck ^ backData[i]
                if serNumCheck != backData[4]:
                    status = self.MI_ERR
            else:
                status = self.MI_ERR

        return (status, backData)

    def CalculateCRC(self, pIndata):
        self.ClearBitMask(self.DivIrqReg, 0x04)
        self.SetBitMask(self.FIFOLevelReg, 0x80)

        for i in range(len(pIndata)):
            self.Write_MFRC522(self.FIFODataReg, pIndata[i])

        self.Write_MFRC522(self.CommandReg, self.PCD_CALCCRC)
        i = 0xFF
        while True:
            n = self.Read_MFRC522(self.DivIrqReg)
            i -= 1
            if not ((i != 0) and not (n & 0x04)):
                break
        pOutData = []
        pOutData.append(self.Read_MFRC522(self.CRCResultRegL))
        pOutData.append(self.Read_MFRC522(self.CRCResultRegM))
        return pOutData

    def MFRC522_SelectTag(self, serNum):
        backData = []
        buf = []
        buf.append(self.PICC_SElECTTAG)
        buf.append(0x70)
        
        for i in range(5):
            buf.append(serNum[i])

        pOut = self.CalculateCRC(buf)
        buf.append(pOut[0])
        buf.append(pOut[1])
        (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, buf)

        if (status == self.MI_OK) and (backLen == 0x18):
            self.logger.debug("Size: " + str(backData[0]))
            return backData[0]
        else:
            return 0

    def MFRC522_Auth(self, authMode, BlockAddr, Sectorkey, serNum):
        buff = []

        # First byte should be the authMode (A or B)
        buff.append(authMode)

        # Second byte is the trailerBlock (usually 7)
        buff.append(BlockAddr)

        # Now we need to append the authKey which usually is 6 bytes of 0xFF
        for i in range(len(Sectorkey)):
            buff.append(Sectorkey[i])

        # Next we append the first 4 bytes of the UID
        for i in range(4):
            buff.append(serNum[i])

        # Now we start the authentication itself
        (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_AUTHENT, buff)

        # Check if an error occurred
        if not (status == self.MI_OK):
            self.logger.error("AUTH ERROR!!")
        if not (self.Read_MFRC522(self.Status2Reg) & 0x08) != 0:
            self.logger.error("AUTH ERROR(status2reg & 0x08) != 0")

        # Return the status
        return status

    def MFRC522_StopCrypto1(self):
        self.ClearBitMask(self.Status2Reg, 0x08)

    def MFRC522_Read(self, blockAddr):
        recvData = []
        recvData.append(self.PICC_READ)
        recvData.append(blockAddr)
        pOut = self.CalculateCRC(recvData)
        recvData.append(pOut[0])
        recvData.append(pOut[1])
        (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, recvData)
        if not (status == self.MI_OK):
            self.logger.error("Error while reading!")

        if len(backData) == 16:
            self.logger.debug("Sector " + str(blockAddr) + " " + str(backData))
            return backData
        else:
            return None

    def MFRC522_Write(self, blockAddr, writeData):
        buff = []
        buff.append(self.PICC_WRITE)
        buff.append(blockAddr)
        crc = self.CalculateCRC(buff)
        buff.append(crc[0])
        buff.append(crc[1])
        (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, buff)
        if not (status == self.MI_OK) or not (backLen == 4) or not ((backData[0] & 0x0F) == 0x0A):
            status = self.MI_ERR

        self.logger.debug("%s backdata &0x0F == 0x0A %s" % (backLen, backData[0] & 0x0F))
        if status == self.MI_OK:
            buf = []
            for i in range(16):
                buf.append(writeData[i])

            crc = self.CalculateCRC(buf)
            buf.append(crc[0])
            buf.append(crc[1])
            (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, buf)
            if not (status == self.MI_OK) or not (backLen == 4) or not ((backData[0] & 0x0F) == 0x0A):
                self.logger.error("Error while writing")
            if status == self.MI_OK:
                self.logger.debug("Data written")


    def MFRC522_DumpClassic1K(self, key, uid):
        for i in range(64):
            status = self.MFRC522_Auth(self.PICC_AUTHENT1A, i, key, uid)
            # Check if authenticated
            if status == self.MI_OK:
                self.MFRC522_Read(i)
            else:
                self.logger.error("Authentication error")

    def MFRC522_Init(self):
        self.MFRC522_Reset()

        self.Write_MFRC522(self.TModeReg, 0x8D)
        self.Write_MFRC522(self.TPrescalerReg, 0x3E)
        self.Write_MFRC522(self.TReloadRegL, 30)
        self.Write_MFRC522(self.TReloadRegH, 0)

        self.Write_MFRC522(self.TxASKReg, 0x40)
        self.Write_MFRC522(self.ModeReg, 0x3D)
        self.AntennaOn()
