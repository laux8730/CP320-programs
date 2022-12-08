import time
import RPi.GPIO as GPIO
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.virtual import viewport, sevensegment
from luma.core.render import canvas
from datetime import datetime

GPIO.setmode(GPIO.BCM)

#UltraSonic Pins
gpioTrig = 19
gpioEcho = 26

GPIO.setup(gpioTrig, GPIO.OUT)
GPIO.setup(gpioEcho, GPIO.IN)
SONIC_SPEED = 34300

def scrolling_message(device, msg, delay=0.1):
    # Implemented with virtual viewport
    width = device.width
    padding = " " * width
    msg = padding + msg + padding
    msg_len = len(msg)

    virtual = viewport(device, width=msg_len, height=8)
    sevensegment(virtual).text = msg
    for i in reversed(list(range(msg_len - width))):
        virtual.set_position((i, 0))
        time.sleep(delay)

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


def clockTime(seg, seconds):
    interval = 0.5
    
    for i in range(int(seconds / interval)):
        now = datetime.now()
        seg.text = now.strftime("%H-%M-%S")

        if i % 2 == 0:
            seg.text = now.strftime("%H-%M-%S")
        else:
            seg.text = now.strftime("%H %M %S")

        time.sleep(interval)

#setup for 8 digit 7-segment display
serial = spi(port=0, device=0, gpio=noop())
device = max7219(serial, cascaded=1)
seg = sevensegment(device)


#main code
seg.text = "HELLO"
select = 1
while(select != str(0)):
    select = input(" 1 -> Message\n 2 -> Display numbers\n 3 -> Ultrasonic Display\n 4 -> Custom Scrolling Message\n 5 -> display time\n 6 -> display date\n 0 -> Exit\n Enter Selection: ")
    
    if(select == "1"):
        seg.text = ("Welcome")
        time.sleep(3)

    if(select == "2"):
        numbers = "12345678"
        seg.text = (numbers)
        time.sleep(3)
    
    if(select == "3"):
        distance = ultrasonicDist()
        print("actual distance: ", distance)
        seg.text = ("%.1f away" %distance)
        time.sleep(3)
    
    if(select == "4"):
        message = input("Please enter a message you would like to see scroll: ")
        scrolling_message(device, message)
    
    if(select == "5"):
        duration = 5
        clockTime(seg, duration)
    
    if(select == "6"):
        print("date will be displayed\n")
        now = datetime.now()
        seg.text = now.strftime("%y-%m-%d")
        time.sleep(1)
    print()

print("Program will now close.\n")
GPIO.cleanup()
        
