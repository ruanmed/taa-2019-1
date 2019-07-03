import RPi.GPIO as gpio
import time

# gpio.setmode(gpio.BCM)

# gpio.setup(23, gpio.OUT, pull_up_down = gpio.PUD_DOWN)

while True:
	gpio.setmode(gpio.BCM)
	gpio.setwarnings(False)
	gpio.setup(21, gpio.OUT)
	print ("LED on")
	gpio.output(21, gpio.HIGH)
	time.sleep(2)
	print ("LED off")
	gpio.output(21, gpio.LOW)
	time.sleep(2)

gpio.cleanup()
exit()
