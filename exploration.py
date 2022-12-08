import time
import RPi.GPIO as GPIO
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.virtual import viewport, sevensegment
from luma.core.render import canvas

GPIO.setmode(GPIO.BCM)

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


serial = spi(port=0, device=0, gpio=noop())
device = max7219(serial, cascaded=1)
seg = sevensegment(device)

seg.text = "Init"
time.sleep(1)

seg.text = str(1234)
time.sleep(4)

scrolling_message(device, "HELLO EVERYONE!")