#!/usr/bin/python

#    HAL2Arduino
#    This user HAL space component is meant primarily as a 'breakout' interface
#    between HAL and the Arduino platform.

#    Note: This interface will always have (at least) serveral milliseconds of
#    lag, compared to the standard parallel port latency with lag times of just
#    nanoseconds.

#    Through it is possible to approximate a CNC controller via this interface
#    it is best used when considered as:
#     A.    Experimental.
#     B.    Not intended for production use.
#     C.    Cannot compete with a real controller vs. speed/cost/accuracy.
#     D.    Best thought of as a 'Duct Tape' interface.
#     E.    Should NEVER be used for THREADING or SYNCRONIZED operations.

#    With that said, it IS good for:
#     A.    Bootstrapping DIY toy CNC machines for existing Arduino owners
#               that would just like to "try stuff out".
#     B.    Hobby grade CNC 3-axis wood router/plasma/printing/plotting tables.
#     C.    Temporary addons to existing CNC machines.
#     D.    Can easily be made to automate tool changers.
#     E.    Interfacing of non-timing critical CNC subsystems such as pumps,
#           pendant controls, LCD displays and such.

#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

import sys, string
#We only need some of the functions from the following modules
from thread import start_new_thread, exit
from Queue import Queue
from serial import Serial
from time import sleep
from tkMessageBox import showinfo
from Tkinter import Tk
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
firmwareVersion = 0.1
global c


#set floats to 4 digits of precision. 1/10000ths of an inch (or mm).
getcontext().prec = 4


import hal # LinuxCNC must be up and running pior to loading this.
c = hal.component("HAL2PI")



#GPIO SETUP

GPIO.setup(22, GPIO.OUT)
GPIO.output(22, GPIO.LOW)
c.newpin("gpio_22",hal.HAL_BIT,hal.HAL_IN)

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

print "HAL2PI started!"


while True:
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
	


raise SystemExit



