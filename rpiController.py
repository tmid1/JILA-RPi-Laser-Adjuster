import sys
from typing import Optional
from queue import Queue, Empty
import time
from time import sleep
import vmbpy as vmb
from vmbpy import *
import cv2
import numpy as np
import usb.core
import usb.util
import re
import RPi.GPIO as GPIO
import threading
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import griddata
import lmfit
from lmfit.lineshapes import gaussian2d, lorentzian
from numpy import asarray
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from matplotlib.patches import Circle

# TO RUN: python rpiController.py ttl periodic

#FRAME CENTERS
xCenter = 500
yCenter = 500

class Handler:
    def __init__(self):
		#If you are expecting to have x amount of experiment iterations within 1 sec, you should queue x amount of frames
        self.display_queue = Queue(1)

    def get_image(self):
        return self.display_queue.get()

    def __call__(self, cam: Camera, stream: Stream, frame: Frame):
        if frame.get_status() == FrameStatus.Complete:

            # Convert frame if it is not already the correct format
            display = frame
            self.display_queue.put(display.as_opencv_image(), True)

        cam.queue_frame(frame)

def get_camera(camera_id: Optional[str]) -> Camera:
    with VmbSystem.get_instance() as vmb:
            cams = vmb.get_all_cameras()
            if not cams:
                abort('No Cameras accessible. Abort.')
            return cams[0]

def setup_camera(cam: Camera):
    with cam:
        # Enable auto exposure time setting if camera supports it
        try:
            cam.ExposureAuto.set('Continuous')
            cam.ExposureTime = 0.05
    

        except (AttributeError, VmbFeatureError):
            pass

        # Enable white balancing if camera supports it
        try:
            cam.BalanceWhiteAuto.set('Continuous')

        except (AttributeError, VmbFeatureError):
            pass

        # Try to adjust GeV packet size. This Feature is only available for GigE - Cameras.
        try:
            stream = cam.get_streams()[0]
            stream.GVSPAdjustPacketSize.run()
            while not stream.GVSPAdjustPacketSize.is_done():
                pass

        except (AttributeError, VmbFeatureError):
            pass

def setup_pixel_format(cam: Camera):
    # Query available pixel formats. Prefer color formats over monochrome formats
    cam_formats = cam.get_pixel_formats()
    cam_color_formats = intersect_pixel_formats(cam_formats, COLOR_PIXEL_FORMATS)
    convertible_color_formats = tuple(f for f in cam_color_formats
                                      if opencv_display_format in f.get_convertible_formats())

    cam_mono_formats = intersect_pixel_formats(cam_formats, MONO_PIXEL_FORMATS)
    convertible_mono_formats = tuple(f for f in cam_mono_formats
                                     if opencv_display_format in f.get_convertible_formats())

    # if OpenCV compatible color format is supported directly, use that
    if opencv_display_format in cam_formats:
        cam.set_pixel_format(opencv_display_format)

    # else if existing color format can be converted to OpenCV format do that
    elif convertible_color_formats:
        cam.set_pixel_format(convertible_color_formats[0])

    # fall back to a mono format that can be converted
    elif convertible_mono_formats:
        cam.set_pixel_format(convertible_mono_formats[0])

    else:
        abort('Camera does not support an OpenCV compatible format. Abort.')

def get_frames():
    with VmbSystem.get_instance():
        print("Getting Frames")
        while True:
            with get_camera(None) as cam:
                # setup general camera settings and the pixel format in which frames are recorded
                setup_camera(cam)
                handler = Handler()
                # Start Streaming with a custom a buffer of 10 Frames (defaults to 5)
                cam.start_streaming(handler=handler, buffer_count=1)

                global done
                if done == True:
                  break

                global shared_frame
                shared_frame = None
                frame = handler.get_image()
                shared_frame = frame
                cam.stop_streaming()
                sleep(0.01)

class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback

        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """
        error = self.SetPoint - feedback_value
        print("PID Error:", error)

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time

def wait_for_it():
    level = GPIO.input(13)
    while (GPIO.input(13)) :
        time.sleep(.001)

NEWFOCUS_COMMAND_REGEX = re.compile("([0-9]{0,1})([a-zA-Z?]{2,})([0-9+-]*)")
MOTOR_TYPE = {
        "0":"No motor connected",
        "1":"Motor Unknown",
        "2":"'Tiny' Motor",
        "3":"'Standard' Motor"
        }

class Controller(object):
    """Picomotor Controller

    Example:

        >>> controller = Controller(idProduct=0x4000, idVendor=0x104d)
        >>> controller.command('VE?')

        >>> controller.start_console()
    """


    def __init__(self, idProduct, idVendor):
        """Initialize the Picomotor class with the spec's of the attached device

        Call self._connect to set up communication with usb device and endpoints

        Args:
            idProduct (hex): Product ID of picomotor controller
            idVendor (hex): Vendor ID of picomotor controller
        """
        self.idProduct = idProduct
        self.idVendor = idVendor
        self._connect()


    def _connect(self):
        """Connect class to USB device

        Find device from Vendor ID and Product ID
        Setup taken from [1]

        Raises:
            ValueError: if the device cannot be found by the Vendor ID and Product
                ID
            Assert False: if the input and outgoing endpoints can't be established
        """
        # find the device
        self.dev = usb.core.find(idVendor=0x104d, idProduct=0x4000)

        if self.dev is None:
            raise ValueError('Device not found')

        # set the active configuration. With no arguments, the first
        # configuration will be the active one
        self.dev.set_configuration()

        # get an endpoint instance
        cfg = self.dev.get_active_configuration()
        intf = cfg[(0,0)]

        self.ep_out = usb.util.find_descriptor(
            intf,
            # match the first OUT endpoint
            custom_match = \
            lambda e: \
                usb.util.endpoint_direction(e.bEndpointAddress) == \
                usb.util.ENDPOINT_OUT)

        self.ep_in = usb.util.find_descriptor(
            intf,
            # match the first IN endpoint
            custom_match = \
            lambda e: \
                usb.util.endpoint_direction(e.bEndpointAddress) == \
                usb.util.ENDPOINT_IN)

        assert (self.ep_out and self.ep_in) is not None

        # Confirm connection to user
        resp = self.command('VE?')




    def send_command(self, usb_command, get_reply=False):
        """Send command to USB device endpoint

        Args:
            usb_command (str): Correctly formated command for USB driver
            get_reply (bool): query the IN endpoint after sending command, to
                get controller's reply

        Returns:
            Character representation of returned hex values if a reply is
                requested
        """
        self.ep_out.write(usb_command)
        if get_reply:
            return self.ep_in.read(100)


    def parse_command(self, newfocus_command):
        """Convert a NewFocus style command into a USB command

        Args:
            newfocus_command (str): of the form xxAAnn
                > The general format of a command is a two character mnemonic (AA).
                Both upper and lower case are accepted. Depending on the command,
                it could also have optional or required preceding (xx) and/or
                following (nn) parameters.
                cite [2 - 6.1.2]
        """
        m = NEWFOCUS_COMMAND_REGEX.match(newfocus_command)

        # Check to see if a regex match was found in the user submitted command
        if m:

            # Extract matched components of the command
            driver_number, command, parameter = m.groups()


            usb_command = command

            # Construct USB safe command
            if driver_number:
                usb_command = '1>{driver_number} {command}'.format(
                                                    driver_number=driver_number,
                                                    command=usb_command
                                                    )
            if parameter:
                usb_command = '{command} {parameter}'.format(
                                                    command=usb_command,
                                                    parameter=parameter
                                                    )

            usb_command += '\r'

            return usb_command
        else:
            print("ERROR! Command {} was not a valid format".format(
                                                            newfocus_command
                                                            ))


    def parse_reply(self, reply):
        """Take controller's reply and make human readable

        Args:
            reply (list): list of bytes returns from controller in hex format

        Returns:
            reply (str): Cleaned string of controller reply
        """

        # convert hex to characters
        reply = ''.join([chr(x) for x in reply])
        return reply.rstrip()


    def command(self, newfocus_command):
        """Send NewFocus formated command

        Args:
            newfocus_command (str): Legal command listed in usermanual [2 - 6.2]

        Returns:
            reply (str): Human readable reply from controller
        """
        usb_command = self.parse_command(newfocus_command)

        # if there is a '?' in the command, the user expects a response from
        # the driver
        if '?' in newfocus_command:
            get_reply = True
        else:
            get_reply = False

        reply = self.send_command(usb_command, get_reply)

        # if a reply is expected, parse it
        if get_reply:
            return self.parse_reply(reply)


    def start_console(self):
        """Continuously ask user for a command
        """
        print('''
        Picomotor Command Line
        ---------------------------

        Enter a valid NewFocus command, or 'quit' to exit the program.

        Common Commands:
            xMV[+-]: .....Indefinitely move motor 'x' in + or - direction
                 ST: .....Stop all motor movement
              xPRnn: .....Move motor 'x' 'nn' steps
        \n
        ''')

        while True:
            command = input("Input > ")
            if command.lower() in ['q', 'quit', 'exit']:
                break
            else:
                rep = self.command(command)
                if rep:
                    print("Output: {}".format(rep))

# Function to be executed when signal is received or every hour
def perform_action(vmb, controller):
	
	#center of frame
	global yCenter, xCenter
	loop = 0
	xDone = False
	yDone = False
	
	targetX = xCenter
	targetY = yCenter
	
	Py = 1.24 
	Iy = 0.53455 
	Dy = 0.471
	
	Px = 1.2556
	Ix = 0.904620028
	Dx = 0.47692
	
	pidX = PID(Px, Ix, Dx)
	pidX.SetPoint = targetX
	pidX.setSampleTime(0.1)
	
	pidY = PID(Py, Iy, Dy)
	pidY.SetPoint = targetY
	pidY.setSampleTime(0.1)
	
	#take a photo using vmbpy (This code works for the Mako 130U B camera)
	#calculate converts the frame to binary the moments of the image and the x and y centroids
	
	while True:
	    
	    newFrame = False
	    global shared_frame
	    
	    loop = loop +1
	    
	    with frame_lock:
	        frame = shared_frame
	        newFrame = True
	    
	    try:
	        if newFrame == True:
	            maxB = np.max(frame)
	            _, binary = cv2.threshold(frame,maxB*0.5,255,cv2.THRESH_BINARY)
	            M = cv2.moments(binary)
	            cenX = np.round(M['m10']/M['m00']) #(M10/M00)
	            cenY = np.round(M['m01']/M['m00']) #(M01/M00)
	            
	            print("X, Y Centroids:",cenX, cenY)
	            
	            if((cenX == xCenter) and (cenY ==  yCenter)) or (xDone == True and yDone == True) or (loop==7):
	                break
	                
	            if(cenX == xCenter):
	                xDone = True
	            if(cenY == yCenter):
	                yDone = True
	                    
	            pidX.update(cenX)
	            pidY.update(cenY)
	            pixX = pidX.output
	            pixY = pidY.output
	            
	            moveX= np.round(((pixX)/0.003794 )*(0.7*0.07)/8)
	            moveY = np.round(-((pixY)/0.003794 )*(0.7*0.07)/4)
	            
	            print(moveX, moveY, "move")
	        
	            if(yDone == False):
	                controller.command('2PR' + str(moveY))
	                sleep((moveY*0.000260005) + 0.2)

	            if(xDone == False):
	                controller.command('1PR' + str(moveX))
	                sleep((moveX*0.000260005) + 0.3)  
	    
	    except:
	        pass
	        
	                    
	    sleep(0.3)

# Function to wait for a GPIO signal
def wait_for_signal(event, actionLock, vmb, controller):	
	#infinite loop, where if a ttl signal is received, a the action function is called
	while True:
		# Add event detection on the GPIO pin
	    wait_for_it()
	    with actionLock:
		    actioned = True
		    perform_action(vmb, controller)
		    actioned = False	    
	
# Function to handle periodic action
def periodic_action(event, actionLock, vmb, controller):
	while True:
		print("period")
		# Perform the action periodically every hour
		if actioned == False:
			with actionLock:
				start_time = time.time()
				perform_action(vmb, controller)
				print(time.time()-start_time, "seconds")
		else:
			pass
			
		# Sleep until the next scheduled time (1 hr)
		sleep(3600)

#Function to intialize and center laser at the beginning of the program	
def intialize(vmb, controller):
	#center of frame
	global xCenter, yCenter
	#take a photo using vmbpy (This code works for the Mako 130U B camera)
	cenM_x = 0
	cenM_y = 0
	while cenM_x != xCenter and cenM_y != yCenter:
		with vmb:
			cams = vmb.get_all_cameras()
		
			with cams[0] as cam:
				cam.ExposureTime = 0.05
				frame = cam.get_frame()
				frame = frame.as_opencv_image()
		
		maxB = np.max(frame)
		_, binary = cv2.threshold(frame,maxB*0.5,255,cv2.THRESH_BINARY)
		#calculate converts the frame to binary the moments of the image and the x and y centroids
		M = cv2.moments(binary)
		cenM_x = np.round(M['m10']/M['m00']) #(M10/M00)
		cenM_y = np.round(M['m01']/M['m00']) #(M01/M00)
		
		print(cenM_x, cenM_y)
		
		if cenM_x == xCenter and cenM_y == yCenter:
			print("initalized")
			return
		
		#calculate the number of steps the motors need to move based on the distance(px) from the centerpoint of the frame

		moveX= np.round(((xCenter - cenM_x)/0.003794 )*(0.7*0.07)/8)
		moveY = np.round(-((yCenter - cenM_y)/0.003794 )*(0.7*0.07)/4)
		
		if(cenM_x != xCenter): #if the x is not already centered
			
			#format the amount of steps to move as a string and send the command to motor controller in Newport motor controller format ("Motor# PR #steps")
			text = '1PR' +str(moveX) 
			controller.command(text)
			
			#wait for the controller to be done moving in the x direction, it cannot take commands while in motion
			sleep((moveX*0.00000160005) + 1)
			
		if(cenM_y != yCenter): #if the y is not already centered
			
			#command for other motor, y direction
			text = '2PR'+str(moveY)
			controller.command(text)

#Function to check args of command line and run threads accordingly
def checkArgs(signal_thread, action_thread):
	
	if(len(sys.argv) == 1):
		# Create and start the thread that handles the periodic action
		print("Starting WITHOUT periodic actions and WITHOUT TTL INPUT.")
		
	elif(len(sys.argv) >=2):
		if(sys.argv[1] == "periodic"):
			action_thread.start()
			action = True
			print("Starting WITH periodic actions.")
			
		if(sys.argv[1] == "ttl"):
			# Create and start the thread that waits for the GPIO signal, target is the function that is being called in the thread, and args are function arguments
			signal_thread.start()
			signal = True
			print("Starting WITH TTL input actions.")
			
		if((len(sys.argv)) >= 3):
			if(sys.argv[2] == "periodic"):
				action_thread.start()
				action = True
				print("Starting WITH periodic actions.")
				
			if(sys.argv[2] == "ttl"):
				# Create and start the thread that waits for the GPIO signal, target is the function that is being called in the thread, and args are function arguments
				signal_thread.start()
				signal = True
				print("Starting WITH TTL input actions.")
			
			else:
				print ("One or more flags not recognized. Starting WITHOUT periodic actions and/or WITHOUT TTL.")
				
	else:
		print ("One or more flags not recognized. Starting WITHOUT periodic actions and/or WITHOUT TTL.")

#define global variables
done = False
signal = False
action = False
actioned = False
shared_frame = None

#define controller object for communication with motors
controller = Controller(idProduct=0x4000, idVendor=0x104d)

#define the camera instance
vmb = vmb.VmbSystem.get_instance()

#define and set up the GPIO pin that will be receiving the TTL for input
TTL_PIN = 13 #number of the GPIO pin that TTL is on NOT the overall pin number
GPIO.setmode(GPIO.BCM)
GPIO.setup(TTL_PIN, GPIO.IN)

#set the laser to the center
intialize(vmb, controller)

#create a lock, where threads must have the lock in order to perform the action
action_lock = threading.Lock()
frame_lock = threading.Lock()

# Create the event, allows threads to run
signal_event = threading.Event()

# Create and start the thread that handles the async frame capture
frame_thread = threading.Thread(target=get_frames)
frame_thread.start()

sleep(5)

signal_thread = threading.Thread(target=wait_for_signal, args=(signal_event, action_lock, vmb,controller))
action_thread = threading.Thread(target=periodic_action, args=(signal_event,action_lock,vmb,controller))

checkArgs(signal_thread, action_thread)

# Wait for both threads to complete (they won't, because they're in infinite loops)
frame_thread.join()
print("Frame thread joined")
print("NO LONGER TAKING PHOTOS")

if(action):
	action_thread.join()
	print("Action thread joined")

if(signal):
	signal_thread.join()
	print("Signal thread joined")

