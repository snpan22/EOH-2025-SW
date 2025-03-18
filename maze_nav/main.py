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

# GPIO Setup for IR transmitter and Encoders
IR_EMIT0 = 22  
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

#configure internal pull up resistor for specified pins. 
#pulls up pins HIGH by default, will read high unless something pulls it low
# rpi has floating inputs if not actively driven HIGH or LOW
GPIO.setup([MR_OUTA, MR_OUTB, ML_OUTA, ML_OUTB], GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Encoder callback functions
#check B when A changes to determine direction
#because two signals are slighly out of phase (A and B)

def read_encoder_right(channel):
    global pos_right
    if GPIO.input(MR_OUTB):
        pos_right += 1 # motor moving forward
    else:
        pos_right -= 1 # motor moving backwards

def read_encoder_left(channel):
    global pos_left
    if GPIO.input(ML_OUTB):
        pos_left += 1
    else:
        pos_left -= 1

# Attach interrupt handlers
#triggers when OUTA transititions from LOW to HIGH
GPIO.add_event_detect(MR_OUTA, GPIO.RISING, callback=read_encoder_right)
GPIO.add_event_detect(ML_OUTA, GPIO.RISING, callback=read_encoder_left)


pos_left, pos_right = 0, 0  # Encoder positions
prev_t = time.time() #
eprev_left, eprev_right = 0, 0
eintegral_left, eintegral_right = 0, 0

# Motion states
FORWARD = 0
TURN_180 = 1
state = FORWARD

# PID constants
kp = 1.0
kd = 0.025
ki = 0.0

# Target positions for different motions
target_forward = 1000  # Adjust for desired travel distance
target_turn = 500  # Adjust for a 180-degree turn

#GPIO.setup(3, GPIO.OUT)
value = 1
try:
    while True:
        curr_t = time.time()
        dt = curr_t - prev_t
        prev_t = curr_t
        
        if state == FORWARD:
            target_left = target_forward
            target_right = target_forward
        elif state == TURN_180:
            target_left = target_turn
            target_right = -target_turn
            
        def PID(target, pos, eprev, eintegral):
            e = pos - target #!potentially do target - pos
            dedt = (e - eprev)/dt
            eintegral +=e*dt
            u = kp*e + kd*dedt + ki*eintegral
            
            
        u_left,eprev_left, eintegral_left = PID(target_left, pos_left, eprev_left, eintegral_left)
        u_right,eprev_right, eintegral_right = PID(target_right, pos_right, eprev_right, eintegral_right)

        def mpc(u):
            pwr = min(abs(u), 255) #limit power to 255
            dir = 1 if u > 0 else -1
            return dir, int(pwr)
        
        dir_left, pwm_left = mpc(u_left)
        dir_right, pwm_right = mpc(u_right)
        command = f"{dir_left},{pwm_left},{dir_right},{pwm_right}\n"
        teensy.write(command.encode())
        
        if abs(target_left - pos_left) < 10 and abs(target_right - pos_right) < 10:
            state = TURN_180 if state == FORWARD else FORWARD
            pos_left, pos_right = 0, 0  # Reset encoder positions

        # Print debug info
        print(f"Target: {target_left}, {target_right} | Pos: {pos_left}, {pos_right} | PWM: {pwm_left}, {pwm_right}")
        time.sleep(0.01)  # Small delay
except KeyboardInterrupt:
    print("Stopping...")
    GPIO.cleanup()
    
    
    
    
    # GPIO.setup(MR_OUTB, GPIO.IN)
# GPIO.setup(ML_OUTB, GPIO.IN)
# GPIO.setup(ML_OUTA, GPIO.IN)
# GPIO.setup(MR_OUTA, GPIO.IN)
            
        # right_b = GPIO.input(MR_OUTB)
        # right_a = GPIO.input(MR_OUTA)
        # left_b = GPIO.input(ML_OUTB)
        # left_a = GPIO.input(ML_OUTA)

        # # Turn on IR emitter (send pulse)
        # GPIO.output(IR_EMIT0, GPIO.HIGH)
        # GPIO.output(IR_EMIT1, GPIO.HIGH)
        # GPIO.output(IR_EMIT2, GPIO.HIGH)
        # GPIO.output(IR_EMIT3, GPIO.HIGH)
        # #time.sleep(3)  # Small delay to allow reflection
       
        # # Read ADC value
        # vz = channel0.voltage
        # vo = channel1.voltage
        # vtw = channel2.voltage
        # vth = channel3.voltage
        # #Print values
        # print("==========================================")
        # print(f'Receiver 0 Raw ADC Value: {channel0.value}')
        # print(f'Receiver 0 ADC Voltage: {vz:.3f} V')
        # print(f'Receiver 1 Raw ADC Value: {channel1.value}')
        # print(f'Receiver 1 ADC Voltage: {vo:.3f} V')
        # print(f'Receiver 2 Raw ADC Value: {channel2.value}')
        # print(f'Receiver 2 ADC Voltage: {vtw:.3f} V')
        # print(f'Receiver 3 Raw ADC Value: {channel3.value}')
        # print(f'Receiver 3 ADC Voltage: {vth:.3f} V')
        # print("==========================================\n\n")
        # teensy.write(bytes([1]))
        # print("sent")
        # # print("MR_OUTA: ", right_a)
        # # print("MR_OUTB: ", right_b)
        # # print("ML_OUTA: ", left_a)
        # # print("ML_OUTB: ", left_b)
        # time.sleep(1)