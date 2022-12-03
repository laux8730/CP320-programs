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
SONIC_SPEED = 34300

#GPIO setup
GPIO.setup(gpioServo, GPIO.OUT)
GPIO.setup(gpioTrig, GPIO.OUT)
GPIO.setup(gpioEcho, GPIO.IN)
GPIO.setup(stepper_pins, GPIO.OUT)
GPIO.setwarnings(False)

#set value p as pulse width modulation
p = GPIO.PWM(gpioServo, freq)

#bus = bus.SMBus(1)


def stepperMotor():
    step_sequence = []
    step_sequence.append([1, 0, 0, 0])
    step_sequence.append([0, 1, 1, 0])
    step_sequence.append([0, 0, 1, 1])
    step_sequence.append([1, 0, 0, 1])

    for row in step_sequence:
        GPIO.output(stepper_pins, row)
        time.sleep(1)

def servoMotor(direction, direction2):
    p.start(MIDDLE)
    p.ChangeDutyCycle(direction)
    time.sleep(1)
    p.ChangeDutyCycle(direction2)
    time.sleep(1)

def ultrasonic():
    # set Trigger to HIGH
  GPIO.output(gpioTrig, True)

  # set Trigger after 0.01ms to LOW
  time.sleep(0.00001)
  GPIO.output(gpioTrig, False)

  StartTime = time.time()
  StopTime = time.time()

  # save StartTime
  while GPIO.input(gpioEcho) == 0:
      StartTime = time.time()

  # save time of arrival
  while GPIO.input(gpioEcho) == 1:
      StopTime = time.time()

  # time difference between start and arrival
  TimeElapsed = StopTime - StartTime
  # multiply with the sonic speed (34300 cm/s)
  # and divide by 2, because there and back
  distance = (TimeElapsed * SONIC_SPEED) / 2

  return distance

def potentiometer():
    bus = smbus.SMBus(1)
    bus.write_byte(ADDRESS,pot)
    value = bus.read_byte(ADDRESS)
    
    return value


def mapToAngle(value, minD, maxD, minA, maxA):
  output = minA + (float(value - minD) / float(maxD - minD) * (maxA - minA))
  return max(min(output, maxA), minA)


response = " "
while (response != "servo" and response != "stepper"):
    response = input("Please enter servo or stepper: ")

if( response == 'servo'):
    while True:
        
        value = potentiometer()
        print(value)
        if(value >= 0 and value < 101):
            servoMotor(RIGHT,LEFT)
        else:
            dist = ultrasonic()
            degrees = mapToAngle(dist, 0, 30, 0, 190)
            angle = LEFT + DEG*degrees
            p.ChangeDutyCycle(angle)
        
        


'''
while True:
    
    dist = ultrasonic()
    print ("Measured Distance = %.1f cm" % dist)
    time.sleep(0.7)
    
    value = potentiometer()
    if(value >= 0 and value <= 100):
        print("The value is between 0 and 100: ", value)
    elif(value > 100 and value <= 200):
        print("The value is between 100 and 200: ", value)
    else:
        print("The value is abover 200: ",  value)
    time.sleep(1)
    
    servoMotor()
'''
GPIO.cleanup()













