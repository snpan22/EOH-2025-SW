import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import lgpio  # Using lgpio instead of RPi.GPIO
import RPi.GPIO as GPIO
from gpiozero import Button
import time
import serial

# Initialize global position variables
pos_right = 0
pos_left = 0

# Initialize serial communication
teensy = serial.Serial("/dev/ttyACM0")
teensy.baudrate = 9600
# ~ GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
# ~ GPIO.setwarnings(False)
# GPIO Pin Definitions
IR_EMIT0 = 22  
IR_EMIT1 = 23 
IR_EMIT2 = 24 
IR_EMIT3 = 25
MR_OUTB = 16
ML_OUTB = 12
ML_OUTA = 5
MR_OUTA = 6

# Initialize lgpio
# ~ h = lgpio.gpiochip_open(0)

# Set up GPIO outputs (IR emitters)
# ~ lgpio.gpio_claim_output(h, IR_EMIT0)
# ~ lgpio.gpio_claim_output(h, IR_EMIT1)
# ~ lgpio.gpio_claim_output(h, IR_EMIT2)
# ~ lgpio.gpio_claim_output(h, IR_EMIT3)

# ~ # Set up GPIO inputs (Encoders) with pull-up resistors
# ~ lgpio.gpio_claim_input(h, MR_OUTA, lgpio.SET_PULL_UP)
# ~ lgpio.gpio_claim_input(h, MR_OUTB, lgpio.SET_PULL_UP)
# ~ lgpio.gpio_claim_input(h, ML_OUTA, lgpio.SET_PULL_UP)
# ~ lgpio.gpio_claim_input(h, ML_OUTB, lgpio.SET_PULL_UP)


# ~ # Encoder callback functions
# ~ def read_encoder_right(chip, gpio, level, timestamp):
    # ~ global pos_right
    # ~ if lgpio.gpio_read(h, MR_OUTB):
        # ~ pos_right += 1  # Motor moving forward
    # ~ else:
        # ~ pos_right -= 1  # Motor moving backward
    # ~ print("right encoder triggered\n")

# ~ def read_encoder_left(chip, gpio, level, timestamp):
    # ~ global pos_left
    # ~ if lgpio.gpio_read(h, ML_OUTB):
        # ~ pos_left += 1
    # ~ else:
        # ~ pos_left -= 1
    # ~ print("left encoder triggered\n")


# ~ # Attach interrupt handlers
# ~ lgpio.callback(h, MR_OUTA, lgpio.RISING_EDGE, read_encoder_right)
# ~ lgpio.callback(h, ML_OUTA, lgpio.RISING_EDGE, read_encoder_left)
# Initialize GPIO


# Set up GPIO outputs (IR emitters)
# ~ GPIO.setup(IR_EMIT0, GPIO.OUT)
# ~ GPIO.setup(IR_EMIT1, GPIO.OUT)
# ~ GPIO.setup(IR_EMIT2, GPIO.OUT)
# ~ GPIO.setup(IR_EMIT3, GPIO.OUT)

# ~ GPIO.setup(MR_OUTB, GPIO.IN)
# ~ GPIO.setup(ML_OUTB, GPIO.IN)
# ~ GPIO.setup(ML_OUTA, GPIO.IN)
# ~ GPIO.setup(MR_OUTA, GPIO.IN)

# Set up encoders as Buttons (pull-up resistors enabled - do not leave floating)
# "Button" is just an object of gpiozero library- think encoders firing (going high) is analagous to pressing a button
MR_OUTA_btn = Button(MR_OUTA, pull_up=True)
MR_OUTB_btn = Button(MR_OUTB, pull_up=True)
ML_OUTA_btn = Button(ML_OUTA, pull_up=True)
ML_OUTB_btn = Button(ML_OUTB, pull_up=True)

def read_encoder_right():
    global pos_right
    if MR_OUTA_btn.is_pressed:  # Read MR_OUTB state
        pos_right += 1  # Forward
    else:
        pos_right -= 1  # Backwards
    # ~ print("right encoder triggered")

def read_encoder_left():
    global pos_left
    if ML_OUTA_btn.is_pressed:  # Read ML_OUTB state
        pos_left -= 1  # Backwards
    else:
        pos_left += 1  # Forwards
    # ~ print("left encoder triggered")


# Attach interrupt handlers
# call these functions when encoder goes high (motion)
#inside callback query outA because outB and outA are phase shifted 90 degrees-->  outA goes high shortly after outB goes high, that way by the time the function is called outA is still high
MR_OUTB_btn.when_pressed = read_encoder_right
ML_OUTB_btn.when_pressed = read_encoder_left

# Setup SPI for MCP3004 ADC
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D8)
mcp = MCP.MCP3008(spi, cs)

# Read from MCP3004
channel0 = AnalogIn(mcp, MCP.P0)  
channel1 = AnalogIn(mcp, MCP.P1)
channel2 = AnalogIn(mcp, MCP.P2)
channel3 = AnalogIn(mcp, MCP.P3)


pos_left, pos_right = 0, 0  # Encoder positions
u_left, u_right = 0, 0
prev_t = time.time() #
eprev_left, eprev_right = 0, 0
eintegral_left, eintegral_right = 0, 0

# Motion states
FORWARD = 0
TURN_180 = 1
STOP = 2
state = FORWARD

# PID constants
kp = 1
kd = 0.025
ki = 0

# Target positions for different motions
target_forward = 1000  # Adjust for desired travel distance
target_turn = 50  # Adjust for a 180-degree turn

#GPIO.setup(3, GPIO.OUT)

try:
    while True:
        # ~ right_b = GPIO.input(MR_OUTB)
        # ~ right_a = GPIO.input(MR_OUTA)
        # ~ left_b = GPIO.input(ML_OUTB)
        # ~ left_a = GPIO.input(ML_OUTA)
        # ~ print("MR_OUTA: ", right_a)
        # ~ print("MR_OUTB: ", right_b)
        # ~ print("ML_OUTA: ", left_a)
        # ~ print("ML_OUTB: ", left_b)

        curr_t = time.time()
        dt = curr_t - prev_t
        prev_t = curr_t
        
        if state == FORWARD:
            target_left = target_forward
            target_right = target_forward
        elif state == TURN_180: # not doing this for now, ignore
            target_left = target_turn
            target_right = -target_turn
        else:
            target_left = 0;
            target_right = 0;
            
        def PID(target, pos, eprev, eintegral):
            e = pos - target #!potentially do target - pos???
            dedt = (e - eprev)/dt
            eintegral +=e*dt
            u = kp*e + kd*dedt + ki*eintegral
            print(u)
            return u, e, eintegral
            
            
        u_left,eprev_left, eintegral_left = PID(target_left, pos_left, eprev_left, eintegral_left)
        u_right,eprev_right, eintegral_right = PID(target_right, pos_right, eprev_right, eintegral_right)

        def mpc(u):
            pwr = min(abs(u), 150) #limit power to 255
            if(pwr <=10):
                dir_ = 0
            else:
                dir_ = 1 if u < 0 else -1
            
            # ~ #left motor: 
            # ~ if(motor): # left motor
                
            # ~ else:
                # ~ #right motor
            # ~ print(u, motor)
            
            
            return dir_, int(pwr)
        
        dir_left, pwm_left = mpc(u_left)
        dir_right, pwm_right = mpc(u_right)
        command = f"{dir_left},{pwm_left},{dir_right},{pwm_right}\n"
        teensy.write(command.encode())
        
        if abs(target_left - pos_left) < 10 and abs(target_right - pos_right) < 10 and state != STOP:
            state = STOP if state == FORWARD else FORWARD
            pos_left, pos_right = 0, 0  # Reset encoder positions

        # Print debug info
        print(f"Target: {target_left}, {target_right} | Pos: {pos_left}, {pos_right} | PWM: {pwm_left}, {pwm_right}")
        time.sleep(0.01)  # Small delay
except KeyboardInterrupt:
    MR_OUTA_btn.close()
    MR_OUTB_btn.close()
    ML_OUTA_btn.close()
    ML_OUTB_btn.close()
    
    # ~ GPIO.cleanup()
    print("Stopping...")
    # ~ lgpio.gpiochip_close(h)

    
    
    
# import busio
# import digitalio
# import board
# import adafruit_mcp3xxx.mcp3008 as MCP
# from adafruit_mcp3xxx.analog_in import AnalogIn
# import RPi.GPIO as GPIO
# import time
# import serial

# teensy = serial.Serial("/dev/ttyACM0")
# teensy.baudrate = 9600

# # Setup SPI for MCP3004 ADC
# spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
# cs = digitalio.DigitalInOut(board.D8)
# mcp = MCP.MCP3008(spi, cs)
# channel0 = AnalogIn(mcp, MCP.P0)  # Read from MCP3004 
# channel1 = AnalogIn(mcp, MCP.P1)
# channel2 = AnalogIn(mcp, MCP.P2)
# channel3 = AnalogIn(mcp, MCP.P3)

# # GPIO Setup for IR transmitter and Encoders
# IR_EMIT0 = 22  
# IR_EMIT1 = 23 
# IR_EMIT2 = 24 
# IR_EMIT3 = 25
# MR_OUTB = 7
# ML_OUTB = 0
# ML_OUTA=5
# MR_OUTA = 6
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(IR_EMIT0, GPIO.OUT)
# GPIO.setup(IR_EMIT1, GPIO.OUT)
# GPIO.setup(IR_EMIT2, GPIO.OUT)
# GPIO.setup(IR_EMIT3, GPIO.OUT)

# #configure internal pull up resistor for specified pins. 
# #pulls up pins HIGH by default, will read high unless something pulls it low
# # rpi has floating inputs if not actively driven HIGH or LOW
# GPIO.setup([MR_OUTA, MR_OUTB, ML_OUTA, ML_OUTB], GPIO.IN, pull_up_down=GPIO.PUD_UP)

# # Encoder callback functions
# #check B when A changes to determine direction
# #because two signals are slighly out of phase (A and B)

# def read_encoder_right(channel):
#     global pos_right
#     if GPIO.input(MR_OUTB):
#         pos_right += 1 # motor moving forward
#     else:
#         pos_right -= 1 # motor moving backwards

# def read_encoder_left(channel):
#     global pos_left
#     if GPIO.input(ML_OUTB):
#         pos_left += 1
#     else:
#         pos_left -= 1

# # Attach interrupt handlers
# #triggers when OUTA transititions from LOW to HIGH
# GPIO.add_event_detect(MR_OUTA, GPIO.RISING, callback=read_encoder_right)
# GPIO.add_event_detect(ML_OUTA, GPIO.RISING, callback=read_encoder_left)
    
    
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
