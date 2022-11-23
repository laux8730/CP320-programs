import time
import RPI.GPIO as GPIO

#servo pin
gpioServo = 17

#ultrasonic pins
gpioTrig = 19
gpioEcho = 26

#StepperMotor pins
IN1 = 12
IN2 = 16
IN3 = 20
IN4 = 21
stepper_pins = [IN1, IN2, IN3, IN4]

#Global variables
FULL_ROTATION = 360
STEP_ANGLE = 5.625
