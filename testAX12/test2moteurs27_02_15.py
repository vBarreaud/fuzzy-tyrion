#!/usr/bin/python
import serial
import time
import Adafruit_BBIO.UART as UART
import Adafruit_BBIO.GPIO as GPIO
 
UART.setup("UART2")


GPIO.setup("P8_10", GPIO.OUT)
GPIO.output("P8_10", GPIO.LOW)
UART2_PORT = "/dev/ttyO2"
UART2_BAUD = 115200 
UART2_TX = "spi0_d0"
UART2_RX = "spi0_sclk"
UART2_TXMUX = 1
UART2_RXMUX = 33

UART2_TXIO = 117
UART2_RXIO = 49

AX_ID = 2
AX_LENGTH = 3
AX_INSTRUCTION = 4
AX_PARAMETER = 5
AX_WRITE = 3
AX_READ = 2
AX_CW_ANGLE_LIMIT_LOW = 0x06
AX_CW_ANGLE_LIMIT_HIGH = 0x07
AX_CCW_ANGLE_LIMIT_LOW = 0x08
AX_CCW_ANGLE_LIMIT_HIGH = 0x09

AX_TORQUE_ENABLE = 0x18
AX_LED = 0x19
AX_STATUS_RETURN_LEVEL = 0x16
AX_GOAL_POSITION_LOW = 0x1E
AX_GOAL_POSITION_HIGH = 0x1F
AX_MOVING_SPEED_LOW = 0x20
AX_MOVING_SPEED_HIGH = 0x21
AX_PRESENT_POSITION_LOW = 0x24
AX_PRESENT_POSITION_HIGH = 0x25
AX_CW_COMPLIANCE_MARGIN = 0x1A
AX_CCW_COMPLIANCE_MARGIN = 0x1B
AX_CW_COMPLIANCE_SLOPE = 0x1C
AX_CCW_COMPLIANCE_SLOPE = 0x1D

ax = serial.Serial(UART2_PORT, UART2_BAUD)

def checksum(packet):
        check = 0
        for i in range(AX_ID, (packet[AX_LENGTH] + 3)):
                check += packet[i]
        return 255 - (check % 256)

def settx():
     GPIO.output("P8_10", GPIO.LOW)


def setrx():
     GPIO.output("P8_10", GPIO.HIGH)	


def txrx(txp, rxp):
        txlength = txp[AX_LENGTH] + 4
        rxlength = 0

        txp[0] = 0xff
        txp[1] = 0xff
        txp[txlength - 1] = checksum(txp)

        settx()
        for i in range(txlength): ax.write(chr(txp[i]))
        setrx()

        if txp[AX_INSTRUCTION] != 254:
                if txp[AX_INSTRUCTION] == AX_READ: rxlength = txp[AX_PARAMETER + 1] + 6
                else: rxlength = 6
				
		
                time.sleep(.2)#.02)
                for x in range(ax.inWaiting()): rxp[x] = ord(ax.read())
                for x in range(txlength + 1): rxp.pop(0)
		#print(rxp[0])		
                if rxp[0]!= 255 and rxp[1] != 255: return -2
                if rxp[rxlength - 1] != checksum(rxp): return -3

                return 1

        return 1

def readbyte(id, address):
        txpacket = [0]*8
        rxpacket = [0]*30

        txpacket[AX_ID] = id
        txpacket[AX_LENGTH] = 4
        txpacket[AX_INSTRUCTION] = AX_READ
        txpacket[AX_PARAMETER] = address
        txpacket[AX_PARAMETER + 1] = 1

        result = txrx(txpacket, rxpacket)
        value = rxpacket[AX_PARAMETER]
        return result, value

def readword(id, address):
        txpacket = [0]*8
        rxpacket = [0]*30
        txpacket[AX_ID] = id
        txpacket[AX_LENGTH] = 4
        txpacket[AX_INSTRUCTION] = AX_READ
        txpacket[AX_PARAMETER] = address
        txpacket[AX_PARAMETER + 1] = 2

        result = txrx(txpacket, rxpacket)
        value =((rxpacket[AX_PARAMETER + 1] << 8) + rxpacket[AX_PARAMETER])
        return result, value

def writebyte(id, address, value):
        txpacket = [0x00]*8
        rxpacket = [0x00]*30

        txpacket[AX_ID] = id
        txpacket[AX_LENGTH] = 0x4
        txpacket[AX_INSTRUCTION] = AX_WRITE
        txpacket[AX_PARAMETER] = address
        txpacket[AX_PARAMETER + 1] = value

        return txrx(txpacket, rxpacket)

def writeword(id, address, value):
        txpacket = [0]*9
        rxpacket = [0]*30

        txpacket[AX_ID] = id
        txpacket[AX_LENGTH] = 0x5
        txpacket[AX_INSTRUCTION] = AX_WRITE
        txpacket[AX_PARAMETER] = address
        txpacket[AX_PARAMETER + 1] = value & 0xff
        txpacket[AX_PARAMETER + 2] = (value & 0xff00) >> 8

        return txrx(txpacket, rxpacket)

value = 0
writebyte(0xFE, 0x18, 0x01)
start = 0
writeword(0xFE, AX_STATUS_RETURN_LEVEL, 2)
writeword(0xFE, AX_CW_ANGLE_LIMIT_LOW , 0)
writeword(0xFE, AX_CW_ANGLE_LIMIT_HIGH, 0)
writeword(0xFE, AX_CCW_ANGLE_LIMIT_LOW, 255)
writeword(0xFE, AX_CCW_ANGLE_LIMIT_HIGH, 0x03)
writeword(0xFE, AX_CW_COMPLIANCE_MARGIN, 20)
writeword(0xFE, AX_CCW_COMPLIANCE_MARGIN, 20)
writeword(0xFE, AX_CW_COMPLIANCE_SLOPE, 128)
writeword(0xFE, AX_CCW_COMPLIANCE_SLOPE, 128)
while 1:
        result, value = readword(0xFE, AX_PRESENT_POSITION_LOW)
	print(result)
	print(value)
        GPIO.output("P8_10", GPIO.LOW)
	time.sleep(.02)
        GPIO.output("P8_10", GPIO.HIGH)

	writeword(0xFE, AX_LED, 0x01)
	writeword(0xFE, AX_MOVING_SPEED_LOW, 1023)
	
	writeword(0xFE, AX_GOAL_POSITION_LOW, 1023)
#	writeword(0xFE, AX_GOAL_POSITION_HIGH, 0)
	
	time.sleep(2)
#print(result)
#print(value)
#if result == 1:
#start = 1
             #   print value
 	writeword(0xFE, AX_GOAL_POSITION_LOW, 0x00)
#	writeword(0xFE, AX_GOAL_POSITION_HIGH, 0)
        #writebyte(0xFE, AX_TORQUE_ENABLE, 1)
  #      writeword(0xFE, AX_MOVING_SPEED_LOW, 0x50)

 #       writeword(0xFE, AX_MOVING_SPEED_HIGH, 0x01)

	writeword(0xFE,AX_LED, 0x00)
        time.sleep(2.5)
print(value)
time.sleep(0.5)

