#!/usr/bin/python
# Terry Sturtevant, May 10, 2017 ; modified by Anson Lau November 5, 2022
import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BCM)
IN1 = 12
IN2 = 16
IN3 = 20
IN4 = 21
FULL_ROTATION = 360
STEP_ANGLE = 5.625
complete_shaft = 0
count = 1

stepper_pins=[IN1, IN2, IN3, IN4]

GPIO.setup(stepper_pins,GPIO.OUT)

stepper_sequence=[]
stepper_sequence.append([1, 0, 0, 0])#half stepping section, added 4 more appends than the original code, 1 = GPIO.HIGH, 0 = GPIO.LOW
stepper_sequence.append([1, 1, 0, 0])
stepper_sequence.append([0, 1, 0, 0])
stepper_sequence.append([0, 1, 1, 0])
stepper_sequence.append([0, 0, 1, 0])
stepper_sequence.append([0, 0, 1, 1])
stepper_sequence.append([0, 0, 0, 1])
stepper_sequence.append([1, 0, 0, 1])



try:
	while True:
		for row in reversed (stepper_sequence):
		
			#for row in stepper_sequence:
				
			
			GPIO.output(stepper_pins,row)
			time.sleep(0.01)
			if(complete_shaft == FULL_ROTATION):
				num_of_steps = count
			else:
				count = count + 1
				complete_shaft = STEP_ANGLE * count
				
			
except KeyboardInterrupt:
	pass

GPIO.cleanup()
print("number of steps: ",num_of_steps)


