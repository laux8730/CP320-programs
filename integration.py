import time
import RPi.GPIO as GPIO
import smbus

GPIO.setmode(GPIO.BCM)

#servo pin
gpioServo = 17
freq = 50

#ultrasonic pins
gpioTrig = 19
gpioEcho = 26

#StepperMotor pins
IN1 = 12
IN2 = 16
IN3 = 20
IN4 = 21
stepper_pins = [IN1, IN2, IN3, IN4]

step_sequence = []
step_sequence.append([1, 0, 0, 0])
step_sequence.append([0, 1, 1, 0])
step_sequence.append([0, 0, 1, 1])
step_sequence.append([1, 0, 0, 1])
#potentiometer vars
ADDRESS = 0X48
pot = 0x43


#Global variables
FULL_ROTATION = 360
STEP_ANGLE = 5.625
LEFT = 0.5/20.0 * 100 # by testing (0.5ms)
RIGHT = 2.5/20.0 * 100 # by testing (2.5ms)
MIDDLE = 1.5/20.0 * 100  # by calculation (1.5ms / 20ms * 100)
DEG = (RIGHT - LEFT) / 190.0 # assume range of server is ~190deg

#GPIO setup
GPIO.setup(gpioServo, GPIO.OUT)
GPIO.setup(gpioTrig, GPIO.OUT)
GPIO.setup(gpioEcho, GPIO.IN)
GPIO.setup(stepper_pins, GPIO.OUT)

#set value p as pulse width modulation
p = GPIO.PWM(gpioServo, freq)

bus = bus.SMBus(1)

p.start(MIDDLE)
count = 0
try:
    while True:
        p.ChangeDutyCycle(LEFT)
        time.sleep(1)
        
        count += 1
except KeyboardInterrupt:
    pass
    print("keyboard interrupt")
GPIO.cleanup()