#!/usr/bin/python

#    HAL2PI



#    Through it is possible to approximate a CNC controller via this interface
#    it is best used when considered as:
#     A.    Experimental.
#     B.    Not intended for production use.
#     C.    Cannot compete with a real controller vs. speed/cost/accuracy.
#     D.    Best thought of as a 'Duct Tape' interface.
#     E.    Should NEVER be used for THREADING or SYNCRONIZED operations.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.


import sys, string
#We only need some of the functions from the following modules
from time import sleep
from decimal import *


import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

#GPIO.setup(2, GPIO.OUT)
#GPIO.output(2, GPIO.HIGH)
#GPIO.output(2, GPIO.LOW)
#GPIO.setup(2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#we can make IO with HAL_IO
#if we read GPIO.IN Setup
#if we write GPIO.OUT Setup

#see command 219

#c.newpin("gpio_2",hal.HAL_BIT,hal.HAL_IO)
#if GPIO.input(2):
#c['gpio_2_input'] = 1
#else:
#c['gpio_2_input'] = 0

#GPIO.add_event_detect(channel, GPIO.RISING)  # add rising edge detection on a channel
#do_something()
#if GPIO.event_detected(channel):

#das Programm pollt zyklisch alle Pins, die vorhanden sind, nach deren Wert.
#welche das sind, wurde bisher mittels der Command und Axisliste abgefragt

#hostCheck=c['iocontrol_user-enable-out']

# To locate the corresponding arduino(s) with acceptible firmware.
# We'll need to scan for any/all of them.
firmware = "HAL-2-PI"
firmwareVersion = 0.2
global c


#set floats to 4 digits of precision. 1/10000ths of an inch (or mm).
getcontext().prec = 4


import hal # LinuxCNC must be up and running pior to loading this.
c = hal.component("HAL2PI")



#GPIO SETUP
c.newpin("gpio_22",hal.HAL_BIT,hal.HAL_IO)

GPIO.setup(23, GPIO.OUT)
GPIO.output(23, GPIO.LOW)
c.newpin("gpio_23",hal.HAL_BIT,hal.HAL_IN)

GPIO.setup(24, GPIO.OUT)
GPIO.output(24, GPIO.LOW)
c.newpin("gpio_24",hal.HAL_BIT,hal.HAL_IN)


#EXAMPLE INPUT

#GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#c.newpin("gpio_24",hal.HAL_BIT,hal.HAL_OUT)

#finishing

sleep(1)
c.ready()

if pin_has_writer('gpio_22'):
    GPIO.setup(22, GPIO.OUT)
    GPIO.output(22, GPIO.LOW)
    gpio_22_out=true
    print('gpio 22 is output')
else:
    GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    gpio_22_out=false
    print('gpio 22 is input')






print "HAL2PI started!"

try:
    while True:
	    if gpio_22_out:
            val=c['gpio_22']
            if val:
                GPIO.output(22,GPIO.HIGH);
            else:
                GPIO.output(22,GPIO.LOW);
	    else:
            if GPIO.input(22):
                c['gpio_22'] = 1
            else:
                c['gpio_22'] = 0        



				
	
    val=c['gpio_22']
    if val:
        GPIO.output(22,GPIO.HIGH);
    else:
        GPIO.output(22,GPIO.LOW);
    val=c['gpio_23']
    if val:
        GPIO.output(23,GPIO.HIGH);
    else:
        GPIO.output(23,GPIO.LOW);
    val=c['gpio_24']
    if val:
        GPIO.output(24,GPIO.HIGH);
    else:
        GPIO.output(24,GPIO.LOW);

#    if GPIO.input(24):
#        c['gpio_24'] = 1
#    else:
#        c['gpio_24'] = 0





except KeyboardInterrupt:
    raise SystemExit	





