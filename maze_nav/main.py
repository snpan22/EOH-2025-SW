import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import RPi.GPIO as GPIO
import time
import serial

teensy = serial.Serial("/dev/ttyACM0")
teensy.baudrate = 9600

# Setup SPI for MCP3004 ADC
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D8)
mcp = MCP.MCP3008(spi, cs)
channel0 = AnalogIn(mcp, MCP.P0)  # Read from MCP3004 
channel1 = AnalogIn(mcp, MCP.P1)
channel2 = AnalogIn(mcp, MCP.P2)
channel3 = AnalogIn(mcp, MCP.P3)

# GPIO Setup for IR transmitter
IR_EMIT0 = 22  # Choose a GPIO pin for digital control
IR_EMIT1 = 23 
IR_EMIT2 = 24 
IR_EMIT3 = 25
MR_OUTB = 7
ML_OUTB = 0
ML_OUTA=5
MR_OUTA = 6
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_EMIT0, GPIO.OUT)
GPIO.setup(IR_EMIT1, GPIO.OUT)
GPIO.setup(IR_EMIT2, GPIO.OUT)
GPIO.setup(IR_EMIT3, GPIO.OUT)

GPIO.setup(MR_OUTB, GPIO.IN)
GPIO.setup(ML_OUTB, GPIO.IN)
GPIO.setup(ML_OUTA, GPIO.IN)
GPIO.setup(MR_OUTA, GPIO.IN)
#GPIO.setup(3, GPIO.OUT)
value = 1
try:
    while True:

        right_b = GPIO.input(MR_OUTB)
        right_a = GPIO.input(MR_OUTA)
        left_b = GPIO.input(ML_OUTB)
        left_a = GPIO.input(ML_OUTA)

        # Turn on IR emitter (send pulse)
        GPIO.output(IR_EMIT0, GPIO.HIGH)
        GPIO.output(IR_EMIT1, GPIO.HIGH)
        GPIO.output(IR_EMIT2, GPIO.HIGH)
        GPIO.output(IR_EMIT3, GPIO.HIGH)
        #time.sleep(3)  # Small delay to allow reflection
       
        # Read ADC value
        vz = channel0.voltage
        vo = channel1.voltage
        vtw = channel2.voltage
        vth = channel3.voltage
        #Print values
        print("==========================================")
        print(f'Receiver 0 Raw ADC Value: {channel0.value}')
        print(f'Receiver 0 ADC Voltage: {vz:.3f} V')
        print(f'Receiver 1 Raw ADC Value: {channel1.value}')
        print(f'Receiver 1 ADC Voltage: {vo:.3f} V')
        print(f'Receiver 2 Raw ADC Value: {channel2.value}')
        print(f'Receiver 2 ADC Voltage: {vtw:.3f} V')
        print(f'Receiver 3 Raw ADC Value: {channel3.value}')
        print(f'Receiver 3 ADC Voltage: {vth:.3f} V')
        print("==========================================\n\n")
        teensy.write(bytes([1]))
        print("sent")
        # print("MR_OUTA: ", right_a)
        # print("MR_OUTB: ", right_b)
        # print("ML_OUTA: ", left_a)
        # print("ML_OUTB: ", left_b)
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopping...")
    GPIO.cleanup()