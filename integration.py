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
LEFT = 0.5/20.0 * 100 
RIGHT = 2.5/20.0 * 100 
MIDDLE = 1.5/20.0 * 100  # by calculation (1.5ms / 20ms * 100)
DEG = (RIGHT - LEFT) / 190.0 # assume range of server is ~190deg
SONIC_SPEED = 34300

#GPIO setup

GPIO.setup(gpioServo, GPIO.OUT)
GPIO.setup(gpioTrig, GPIO.OUT)
GPIO.setup(gpioEcho, GPIO.IN)

GPIO.setup(stepper_pins, GPIO.OUT)


#set value p as pulse width modulation
p = GPIO.PWM(gpioServo, freq)




def stepperMotor():
    step_sequence = []
    step_sequence.append([1, 0, 0, 0])
    step_sequence.append([0, 1, 1, 0])
    step_sequence.append([0, 0, 1, 1])
    step_sequence.append([1, 0, 0, 1])

    return step_sequence

def servoMotor(direction, direction2):
    p.start(MIDDLE)
    p.ChangeDutyCycle(direction)
    time.sleep(1)
    p.ChangeDutyCycle(direction2)
    time.sleep(1)

def ultrasonic():

  GPIO.output(gpioTrig, True)


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


def servoAngle(value, minD, maxD, minA, maxA):
  output = minA + (float(value - minD) / float(maxD - minD) * (maxA - minA))
  return max(min(output, maxA), minA)


response = " "
while (response != "servo" and response != "stepper"):
    response = input("Please enter servo or stepper: ")

if( response == 'servo'):

    while True:
        
        value = potentiometer()
        print("current potentiometer value: ", value)
        if(value >= 0 and value < 101):                  #automatically rotates if pot value is between 0 and 100
            servoMotor(RIGHT,LEFT)
        else:                                            #uses ultrasonic to detect a value to generate a certain angle for the servo to move; basically moves servo based on ultrasonic detection
            distance = ultrasonic()
            degrees = servoAngle(distance, 0, 30, 0, 190)
            angle = LEFT + DEG*degrees
            print("distance: ", distance)
            print("angle: ",angle)
            p.ChangeDutyCycle(angle)
            time.sleep(1)

elif(response == 'stepper'):
    
    while True:
        dist = ultrasonic()
        print("ultrasonic dist: ", dist)
        steps = stepperMotor()
        if(dist < 20):
            
                
            for row in (steps): 
                GPIO.output(stepper_pins,row)
                time.sleep(0.01)
        else:
        
            for row in reversed (steps):
                    
                GPIO.output(stepper_pins,row)
                time.sleep(0.01)
        time.sleep(0.01)


    

    

GPIO.cleanup()













