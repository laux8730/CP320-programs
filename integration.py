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

LEFT = 0.5/20.0 * 100 
RIGHT = 2.5/20.0 * 100 
MIDDLE = 1.5/20.0 * 100  # by calculation (1.5ms / 20ms * 100)
DEG = (RIGHT - LEFT) / 190.0 # assume range of server is ~190deg
SONIC_SPEED = 34300
MIN_ULTRASONIC_DIST = 20
MIN_POT_VAL = 0
MIDDLE_POT_VAL = 125

#GPIO setup
GPIO.setup(gpioServo, GPIO.OUT)
GPIO.setup(gpioTrig, GPIO.OUT)
GPIO.setup(gpioEcho, GPIO.IN)
GPIO.setup(stepper_pins, GPIO.OUT)


#set value p as pulse width modulation
p = GPIO.PWM(gpioServo, freq)



#definitions
def stepperMotorInit():
    step_sequence = []
    step_sequence.append([1, 0, 0, 0])
    step_sequence.append([0, 1, 1, 0])
    step_sequence.append([0, 0, 1, 1])
    step_sequence.append([1, 0, 0, 1])

    return step_sequence

def servoMotorDefault(direction, direction2):  
    p.start(MIDDLE)
    p.ChangeDutyCycle(direction)
    time.sleep(1)
    p.ChangeDutyCycle(direction2)
    time.sleep(1)

def ultrasonicDist():
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

def potentiometerControl():
    bus = smbus.SMBus(1)
    bus.write_byte(ADDRESS,pot)
    value = bus.read_byte(ADDRESS)
    
    return value


def servoAngle(value, minD, maxD, minA, maxA):
  output = minA + (float(value - minD) / float(maxD - minD) * (maxA - minA))
  return max(min(output, maxA), minA)

try:
    response = " "
    while (response != "servo" and response != "stepper"):
        response = input("Please enter servo or stepper: ")

    if( response == 'servo'):

        while True:
            
            value = potentiometerControl()
            print("current potentiometer value: ", value)

            if(value >= MIN_POT_VAL and value < MIDDLE_POT_VAL):      #automatically rotates if pot value is between MIN_POT_VAL = 0 and MIDDLE_POT_VAL = 101
                servoMotorDefault(RIGHT,LEFT)
            else:                        # if potentiometer value is > 101; uses ultrasonic to detect a value to generate a certain angle for the servo to move; basically moves servo based on ultrasonic detection
                distance = ultrasonicDist()
                degrees = servoAngle(distance, 0, 30, 0, 190)
                angle = LEFT + DEG*degrees
                print("distance: ", distance)
                print("angle: ",angle)
                print(" ")
                p.ChangeDutyCycle(angle)
                time.sleep(1)

    elif(response == 'stepper'): #user selects to use stepper motor
        
        while True: #will rotate in a direction based on distance detected by ultrasonic
            dist = ultrasonicDist()
            print("ultrasonic dist: ", dist)
            steps = stepperMotorInit()

            if(dist < MIN_ULTRASONIC_DIST): #MIN_ULTRASONIC_DIST = 20; 
                
                for row in (steps): 
                    GPIO.output(stepper_pins,row)
                    time.sleep(0.01)
            else:
            
                for row in reversed (steps):
                        
                    GPIO.output(stepper_pins,row)
                    time.sleep(0.01)
            time.sleep(0.01)

except KeyboardInterrupt:
    print("\nprogram stopped by user")
    p.ChangeDutyCycle(MIDDLE)
    p.stop
    GPIO.cleanup()













