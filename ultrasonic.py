import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

gpioTrig = 19
gpioEcho = 26

GPIO.setup(gpioTrig, GPIO.OUT)
GPIO.setup(gpioEcho, GPIO.IN)
SONIC_SPEED = 34300

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


distance = ultrasonicDist()
print("distance: ", distance)