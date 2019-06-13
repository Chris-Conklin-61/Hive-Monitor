# HiveMonitor.py 

# This program will be used to gather bee hive sensor data and 
# send the data to a ThingSpeak channel.

# 02/07/2019 - Conklin - Initial Version
# 02/12/2019 - Conklin - Round data to the nearest tenth
# 02/16/2019 - Conklin - Cleanup and write output to a log file
# 02/18/2019 - Conklin - Grab Temperature from the DHT22
# --------- Start of V2
# 02/20/2019 - Conklin - Only check Hive Temp,Humidity at GPIO 12
#                      - Create separate functions for reading temp and humidity
#                      - Throw away bad humidity readings
#                      - Check for and throw away Bad temp readings
#                      - Add Ambient Temp,Humidity at GPIO 6
# --------- Start of V3
# 02/24/2019 - Conklin - Add code to read voltage and power from 1NA219
# 02/25/2019 - Conklin - Add Installed Sensor Indications
# 03/02/2019 - Conklin - Add Right Rear HX711 Load Cell Readings
# 03/04/2019 - Conklin - Add Wifi Signal Quality
# --------- Start of V4
# 03/07/2019 - Conklin - Add Blynk support
# 03/08/2019 - Conklin - Add weight measurements
#					   - Add send to thingspeak switch
#                      - Change BLYNK heartbeat to 30 seconds
# 03/09/2019 - Conklin - Add LCD class and support
# --------- Start of V5
# 03/12/2019 - Conklin - Add Reboot Blynk Button Support
#					   - Add Shutdown Blynk Button Support
#					   - Remove Thingspeak related code
# 03/15/2019 - Conklin - Add Reconnections
# 03/19/2019 - Conklin - Send Reconnect info to LCD
# 03/24/2019 - Conklin - Create a shutdown button
# 03/28/2019 - Conklin - Create a sensor status indication
# --------- Start of V6
# 04/03/2019 - Conklin - Log Individual Load Sensor Readings
#                      - Create Terminal Widget
# --------- Start of V7
# 04/24/2019 - Conklin - Change rounding of Load Sensor Readings to 100ths
#					   - Add a Scale Zero Calibration triggered from the App
# 04/30/2019 - Conklin - Change Data gathering loop from 1 to 2 minutes
# 05/03/2019 - Conklin - Lower shutdown voltage to 10.5 Volts
# 05/15/2019 - Conklin - Send Hive Scale information to the IBM IoT
#             
# ----------------------------------------------------------------------

import os
import glob
import time
import sys
import datetime
import requests
import json
import Adafruit_DHT
import RPi.GPIO as GPIO
from ina219 import INA219
from subprocess import *
import BlynkLib
import wiotp

Hive = 'KA1'				           			# Hive Identifier
BLYNK_AUTH = '869a1416985544b7b94b25418a01bae7' # Blynk Hive Monitor Project Authority
Power_Sensor_Installed = True 					# Poll the INA219 Current Sensor
Hive_Temperature_Sensor_Installed = True		# Poll the Hive DHT22 Sensor
Ambient_Temperature_Sensor_Installed = True 	# Poll the Ambient DHT22 Sensor
Hive_Weight_RR_Sensor_Installed = True  		# Poll the Hive Weight Right Rear Sensor
Hive_Weight_LR_Sensor_Installed = True  		# Poll the Hive Weight Left Rear Sensor
Hive_Weight_LF_Sensor_Installed = True  		# Poll the Hive Weight Left Front Sensor
Hive_Weight_RF_Sensor_Installed = True  		# Poll the Hive Weight Right Front Sensor 
# ---- IBM IOT Information ---------------------------------------
organization = "bjxf7f"
typeId = "Hive_Scale"
deviceId = "HiveMonitorA"
authToken = "MyT0kenA"              
# ---- DHT22 Humidity Sensor Information ---------------------------------------
Humidity_Sensor = Adafruit_DHT.DHT22    # specify the Humidity sensor type
DHT22_H_pin = 6	    				    # gpio pin connected to the DHT22 Hive sensor
DHT22_H_reads = 0                       # Count of sensor reads since last start
DHT22_H_fails = 0						# Count of sensor read fails since last start
DHT22_A_pin = 12	  			        # gpio pin connected to the DHT22 Ambient sensor
# ---- Hive Scale Information ------------------------------------------------------
known_weight = 200						# Initialize known weight value for calibration
known_weight_quarter = 50               # Initial know weight value per load cell
# ---- HX711 RR Load Cell Module Information ---------------------------------------
hx711_RR_dout = 4                      	# gpio (BCM) pin for the dout line of the RR HX711
hx711_RR_sck = 17                      	# gpio (BCM) pin for the sck line of the RR HX711
hx711_RR_gain = 128                    	# gain value for the HX711
hx711_RR_offset = 8643597.0625         	# Offset value of the scale (read with no weight)
hx711_RR_ratio = 30658.5               	# Scale ratio
# ---- HX711 LR Load Cell Module Information ---------------------------------------
hx711_LR_dout = 25                     	# gpio (BCM) pin for the dout line of the LR HX711
hx711_LR_sck = 5                       	# gpio (BCM) pin for the sck line of the LR HX711
hx711_LR_gain = 128                    	# gain value for the HX711
hx711_LR_offset = 8411739.9375         	# Offset value of the scale (read with no weight)
hx711_LR_ratio =  45730.0              	# Scale ratio
# ---- HX711 LF Load Cell Module Information ---------------------------------------
hx711_LF_dout = 22                     	# gpio (BCM) pin for the dout line of the LF HX711
hx711_LF_sck = 23                      	# gpio (BCM) pin for the sck line of the LF HX711
hx711_LF_gain = 128                    	# gain value for the HX711
hx711_LF_offset = 8536637.125          	# Offset value of the scale (read with no weight)
hx711_LF_ratio =  67326.2              	# Scale ratio
# ---- HX711 RF Load Cell Module Information ---------------------------------------
hx711_RF_dout = 18                     	# gpio (BCM) pin for the dout line of the RF HX711
hx711_RF_sck = 27                      	# gpio (BCM) pin for the sck line of the RF HX711
hx711_RF_gain = 128                    	# gain value for the HX711
hx711_RF_offset =  8377546.0           	# Offset value of the scale (read with no weight)
hx711_RF_ratio = 27807.4               	# Scale ratio
# ---- INA219 Voltage and Power Monitor Information -------------------------------
shunt_ohms = 0.1
max_expected_amps = 1.0
Shutdown_Voltage = 10.5 				# Shutdown Voltage Value   
# ---- Data Posting Interval Information ------------------------------------------
lastConnectionTime = time.time()	   	# Track the last connection time
lastUpdateTime = time.time() 		   	# Track the last update time
updateInterval = 120				   	# Update and post data once every 120 seconds
# ---- Wifi Monitoring information -------------------------------------------------
shell_cmd = 'iwconfig wlan0 | grep Link'

# ----------------------------------------------------------------------------------
# ----- Blynk Terminal Widget Constructor
# -----
# ----- Call after the Blynk instance is detup passing the virtual pin for the 
# ----- terminal widget
# ----------------------------------------------------------------------------------
class BlynkTerminal():
	def __init__(self, blynk, vPin):
		self.__blynk = blynk
		self.__vPin = vPin
		
	def clear(self):
		self.__blynk.virtual_write(self.__vPin, 'clr')
		
	def print(self,msg):
		self.__blynk.virtual_write(self.__vPin, msg + "\n")
		
# ----------------------------------------------------------------------------------
# ----- Blynk LCD Widget Constructor
# -----
# ----- Call after the Blynk instance is detup passing the virtual pin for the 
# ----- 2 line LCD widget
# -----
# ----- printlcd(x,y,msg) writes the msg at x position, y position
# -----   x can have a value of 0-15
# -----   y can have a value of 0 or 1
# ----------------------------------------------------------------------------------            
class WidgetLCD():
    def __init__(self, blynk, vPin):
        self.__blynk = blynk
        self.__vPin = vPin

    def clear(self):
        self.__blynk.virtual_write(self.__vPin, 'clr')

    def printlcd(self, x, y, s):
        self.__blynk.virtual_write(self.__vPin, '\0'.join(map(str, ('p', x, y, s))))
        
# ----------------------------------------------------------------------------------
# ----- HX711 Load Cell Constructor
# -----
# ----- Initialize with the data pin, clock pin, and gain to use for the HX711 
# -----    dout: Serial Output pin
# -----    pd_sck: Power Down and serial clock input pin
# -----    gain: set to gain of 128,64,or 32
# ----------------------------------------------------------------------------------    
class HX711:

    def __init__(self, dout=25, pd_sck=24, gain=128):
        self.GAIN = 0
        self.OFFSET = 0
        self.SCALE = 1

        # Setup the gpio pin numbering system
        GPIO.setmode(GPIO.BCM)

        # Set the pin numbers
        self.PD_SCK = pd_sck
        self.DOUT = dout

        # Setup the GPIO Pin as output
        GPIO.setup(self.PD_SCK, GPIO.OUT)

        # Setup the GPIO Pin as input
        GPIO.setup(self.DOUT, GPIO.IN)

        # Power up the chip
        self.power_up()
        self.set_gain(gain)

    def set_gain(self, gain=128):

        try:
            if gain is 128:
                self.GAIN = 3
            elif gain is 64:
                self.GAIN = 2
            elif gain is 32:
                self.GAIN = 1
        except:
            self.GAIN = 3  # Sets default GAIN at 128

        GPIO.output(self.PD_SCK, False)
        self.read()

    def set_scale(self, scale):
        """
        Set scale
        :param scale, scale
        """
        self.SCALE = scale

    def set_offset(self, offset):
        """
        Set the offset
        :param offset: offset
        """
        self.OFFSET = offset

    def get_scale(self):
        """
        Returns value of scale
        """
        return self.SCALE

    def get_offset(self):
        """
        Returns value of offset
        """
        return self.OFFSET

    def read(self):
        """
        Read data from the HX711 chip
        :param void
        :return reading from the HX711
        """

        # Control if the chip is ready
        while not (GPIO.input(self.DOUT) == 0):
            # Uncommenting the print below results in noisy output
            # print("No input from HX711.")
            pass

        # Original C source code ported to Python as described in datasheet
        # https://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/hx711_english.pdf
        # Output from python matched the output of
        # different HX711 Arduino library example
        # Lastly, behaviour matches while applying pressure
        # Please see page 8 of the PDF document

        count = 0

        for i in range(24):
            GPIO.output(self.PD_SCK, True)
            count = count << 1
            GPIO.output(self.PD_SCK, False)
            if(GPIO.input(self.DOUT)):
                count += 1

        GPIO.output(self.PD_SCK, True)
        count = count ^ 0x800000
        GPIO.output(self.PD_SCK, False)

        # set channel and gain factor for next reading
        for i in range(self.GAIN):
            GPIO.output(self.PD_SCK, True)
            GPIO.output(self.PD_SCK, False)

        return count

    def read_average(self, times=16):
        """
        Calculate average value from
        :param times: measure x amount of time to get average
        """
        sum = 0
        for i in range(times):
            sum += self.read()
        return sum / times

    def get_lbs(self, times=16):
        """
        :param times: Set value to calculate average, 
        be aware that high number of times will have a 
        slower runtime speed.        
        :return float weight in lbs
        """
        value = (self.read_average(times) - self.OFFSET)
        lbs = (value / self.SCALE)
        return lbs

    def tare(self, times=16):
        """
        Tare functionality fpr calibration
        :param times: set value to calculate average
        """
        sum = self.read_average(times)
        self.set_offset(sum)

    def power_down(self):
        """
        Power the chip down
        """
        GPIO.output(self.PD_SCK, False)
        GPIO.output(self.PD_SCK, True)

    def power_up(self):
        """
        Power the chip up
        """
        GPIO.output(self.PD_SCK, False)
        
# ----------------------------------------------------------------------------------
# ----- Log to data
# ----------------------------------------------------------------------------------            
def LogData(voltage,power,weight,Hive_Humidity,Ambient_Humidity,Hive_Temperature,Ambient_Temperature,wifi):
	
	message = {}
	message['created_at'] = datetime.datetime.now().isoformat()+"-05:00"
	message['A_Temp'] = Ambient_Temperature
	message['A_Humidity'] = Ambient_Humidity
	message['H_Temp'] = Hive_Temperature
	message['H_Humidity'] = Hive_Humidity
	message['Weight'] = weight
	message['Voltage'] = voltage
	message['Power'] = power 
	message['Wifi'] = wifi		   
	log.write(str(message))
	log.write("\n")
	log.flush()
	
# ----------------------------------------------------------------------------------
# ----- Exit Routine
# ----------------------------------------------------------------------------------  
def cleanAndExit():
    log.write("Cleaning...\n")
    GPIO.cleanup()
    log.write("Bye!\n")
    log.close()
    sys.exit()		
    
# ----------------------------------------------------------------------------------
# ----- Read the DHT22 Temperature / Humidty Sensor
# ----------------------------------------------------------------------------------  
def read_DHT22(sensor,pin):
	Humidity, Temperature = Adafruit_DHT.read_retry(sensor,pin,retries=8)
	return(Humidity, Temperature)

# ----------------------------------------------------------------------------------
# ----- Read the wifi signal strength
# ----------------------------------------------------------------------------------  	
def read_wifi():
	sl = 0
	proc = Popen(shell_cmd, shell=True, stdout=PIPE, stderr=PIPE)
	output, err = proc.communicate()
	msg = output.decode('utf-8').strip()
	start = msg.find('Signal level=')
	if start > 0:
		end = msg.find('dBm')
		if end > 0:
			sl = int(msg[start+13:end])	
	return(sl)
	
#----------------------------------------------------------------------------------------#	
#----------------------------------------------------------------------------------------#
#-----                                                                               ----#   
#-----                                                                               ----#
#-----          M A I N    P R O G R A M    S T A R T S    H E R E                   ----#
#-----                                                                               ----#
#-----                                                                               ----#  
#----------------------------------------------------------------------------------------#
#----------------------------------------------------------------------------------------#

	
#-----------------------------------------------------------
# -- Open the log file, Write the Start Header
#-----------------------------------------------------------

log = open("HiveMonitor.log","a")
log.write("-------------------------------------------------------------\n")
log.write("Hive Monitor Started at " + datetime.datetime.now().isoformat() + "-05:00\n")
log.write("-------------------------------------------------------------\n")
log.flush()

#-----------------------------------------------------------
# -- Wait for PI to come up
#-----------------------------------------------------------

time.sleep(60)

#-----------------------------------------------------------
# -- initialize the json data structure
#-----------------------------------------------------------

if os.path.isfile('HiveMonitorData.txt'):
	
	with open('HiveMonitorData.txt') as jsonfile:
		data = json.load(jsonfile)
		
	for load_cells in data['scale']:
		if load_cells['location'] == 'RR':
			hx711_RR_offset = load_cells['offset']
			hx711_RR_ratio = load_cells['ratio']
		elif load_cells['location'] == 'LR':
			hx711_LR_offset = load_cells['offset']
			hx711_LR_ratio = load_cells['ratio']	
		elif load_cells['location'] == 'RF':
			hx711_RF_offset = load_cells['offset']
			hx711_RF_ratio = load_cells['ratio']	
		elif load_cells['location'] == 'LF':
			hx711_LF_offset = load_cells['offset']
			hx711_LF_ratio = load_cells['ratio']
		else:
			log.write("Bad Data in HiveMonitorData.txt!\n")
			print("Bad Data in HiveMonitorData.txt!")
else:
	
	data = {}
	data['scale'] = []

	data['scale'].append({
		'location' : 'RR',
		'offset' : hx711_RR_offset,
		'ratio' : hx711_RR_ratio,
	})

	data['scale'].append({
		'location' : 'RF',
		'offset' : hx711_RF_offset,
		'ratio' : hx711_RF_ratio,
	})

	data['scale'].append({
		'location' : 'LF',
		'offset' : hx711_LF_offset,
		'ratio' : hx711_LF_ratio,
	})

	data['scale'].append({
		'location' : 'LR',
		'offset' : hx711_LR_offset,
		'ratio' : hx711_LR_ratio,
	})

	with open('HiveMonitorData.txt','w') as outfile:
		json.dump(data,outfile)	
	
#-----------------------------------------------------------
# -- Create the Right Rear Scale Instance
#-----------------------------------------------------------

weight_RR_lbs = 0 
if Hive_Weight_RR_Sensor_Installed:
	weight_RR = HX711(dout=hx711_RR_dout, pd_sck=hx711_RR_sck, gain=hx711_RR_gain)
	weight_RR.set_offset(hx711_RR_offset)
	weight_RR.set_scale(hx711_RR_ratio)
	weight_RR.power_down()
	
#-----------------------------------------------------------
# -- Create the Left Rear Scale Instance
#-----------------------------------------------------------

weight_LR_lbs = 0 
if Hive_Weight_LR_Sensor_Installed:
	weight_LR = HX711(dout=hx711_LR_dout, pd_sck=hx711_LR_sck, gain=hx711_LR_gain)
	weight_LR.set_offset(hx711_LR_offset)
	weight_LR.set_scale(hx711_LR_ratio)
	weight_LR.power_down()
	
#-----------------------------------------------------------
# -- Create the Left Front Scale Instance
#-----------------------------------------------------------

weight_LF_lbs = 0 
if Hive_Weight_LF_Sensor_Installed:
	weight_LF = HX711(dout=hx711_LF_dout, pd_sck=hx711_LF_sck, gain=hx711_LF_gain)
	weight_LF.set_offset(hx711_LF_offset)
	weight_LF.set_scale(hx711_LF_ratio)
	weight_LF.power_down()
	
#-----------------------------------------------------------
# -- Create the Left Front Scale Instance
#-----------------------------------------------------------

weight_RF_lbs = 0 
if Hive_Weight_RF_Sensor_Installed:
	weight_RF = HX711(dout=hx711_RF_dout, pd_sck=hx711_RF_sck, gain=hx711_RF_gain)
	weight_RF.set_offset(hx711_RF_offset)
	weight_RF.set_scale(hx711_RF_ratio)
	weight_RF.power_down()

#-----------------------------------------------------------
# -- Create the ina219 Instance
#-----------------------------------------------------------

voltage = 0.0
power = 0.0
if Power_Sensor_Installed:
	ina = INA219(shunt_ohms,max_expected_amps)
	ina.configure(ina.RANGE_16V)
	
#-----------------------------------------------------------
# -- Initialize the Temperature and Humidity data
#-----------------------------------------------------------

Ambient_Temperature = 0.0
Ambient_Humidity = 0.0
Hive_Temperature = 0.0
Hive_Humidity = 0.0

#-----------------------------------------------------------
# -- Create the Blynk Instance
#-----------------------------------------------------------
blynk = BlynkLib.Blynk(BLYNK_AUTH,heartbeat=45)

#-----------------------------------------------------------
# -- Create the Blynk LCD Instance (Connected to V0)
#-----------------------------------------------------------
lcd = WidgetLCD(blynk, 0)
lcd.clear()
lcd.printlcd(0, 0, 'Reconnected at:')
lcd.printlcd(0, 1, datetime.datetime.now().strftime("%m/%d %H:%M:%S"))	

#-----------------------------------------------------------
# -- Create the Blynk Terminal Instance (Connected to V10)
#-----------------------------------------------------------
terminal = BlynkTerminal(blynk, 10)
terminal.clear()
terminal.print('Hive Monitor started at: ' + str(datetime.datetime.now().strftime("%m/%d/%Y %H:%M:%S")))

#-----------------------------------------------------------
# -- Create the IBM IoT Device Instance
#-----------------------------------------------------------
try:
    deviceOptions = {
        "identity": {"orgId": organization, "typeId": typeId, "deviceId": deviceId},
        "auth": {"token": authToken},
    }
    deviceCli = wiotp.sdk.device.DeviceClient(deviceOptions)
except Exception as e:
    print("Caught exception connecting device: %s" % str(e))
    sys.exit()

deviceCli.connect()

#-----------------------------------------------------------
# -- Register Virtual Pin and action for the Reboot Button
# -- Blynk Widget (Connected to V16). Also send a '0' to 
# -- the widget as we are running now.
#-----------------------------------------------------------

@blynk.VIRTUAL_WRITE(16)
def v16_write_handler(value):
	if value[0] == '1':
		log.write("Reboot Button Pressed on Blynk App!\n")
		log.flush()
		print('Reboot Button Pressed on Blynk App')
		terminal.print('Hive Monitor is Rebooting')
		time.sleep(5)
		call("sudo reboot", shell=True)
			
blynk.virtual_write(16,'0')     # Tell the Blynk App we are running

#-----------------------------------------------------------
# -- Register Virtual Pin and action for the Shutdown Button
# -- Blynk Widget (Connected to V17). Also send a '0' to to
# -- the widget as we are running now
#-----------------------------------------------------------

@blynk.VIRTUAL_WRITE(17)
def v17_write_handler(value):
	if value[0] == '1':
		log.write("ShutdownButton Pressed on Blynk App!\n")
		log.flush()
		print('Shutdown Button Pressed on Blynk App')
		terminal.print('Hive Monitor is shutting down')
		time.sleep(5)
		call("sudo shutdown -h", shell=True)
		
blynk.virtual_write(17,'0')     # Tell the Blynk App we are running		

#-----------------------------------------------------------
# -- Register Virtual Pin and action for the Zer0 Calibrate Button
# -- Blynk Widget (Connected to V19). 
#-----------------------------------------------------------

@blynk.VIRTUAL_WRITE(19)
def v19_write_handler(value):
	if value[0] == '1':
		log.write("Zero Calibrate Button Pressed on Blynk App!\n")
		log.flush()
		print('Zero Calibrate Button Pressed on Blynk App')
		terminal.print('Performing a Zero Calibration function')
		
#
# ------ Get New Right Rear zero Offset Reading
#
		if Hive_Weight_RR_Sensor_Installed:
			weight_RR.power_up()	
			hx711_RR_offset = weight_RR.read_average()
			weight_RR.power_down()
			terminal.print('RR Load Cell zero (Offset) => ' + str(hx711_RR_offset))
			print('RR Load Cell zero (Offset) => ' + str(hx711_RR_offset))
			weight_RR.set_offset(hx711_RR_offset)
			
#
# ------ Get New Left Rear zero Offset Reading
#
		if Hive_Weight_LR_Sensor_Installed:
			weight_LR.power_up()	
			hx711_LR_offset = weight_LR.read_average()
			weight_LR.power_down()
			terminal.print('LR Load Cell zero (Offset) => ' + str(hx711_LR_offset))
			print('LR Load Cell zero (Offset) => ' + str(hx711_LR_offset))
			weight_LR.set_offset(hx711_LR_offset)
#
# ------ Get New Left Front zero Offset Reading
#
		if Hive_Weight_LF_Sensor_Installed:
			weight_LF.power_up()	
			hx711_LF_offset = weight_LF.read_average()
			weight_LF.power_down()
			terminal.print('LF Load Cell zero (Offset) => ' + str(hx711_LF_offset))
			print('LF Load Cell zero (Offset) => ' + str(hx711_LF_offset))
			weight_LF.set_offset(hx711_LF_offset)
						
#
# ------ Get New Right Front zero Offset Reading
#
		if Hive_Weight_RF_Sensor_Installed:
			weight_RF.power_up()	
			hx711_RF_offset = weight_RF.read_average()
			weight_RF.power_down()
			terminal.print('RF Load Cell zero (Offset) => ' + str(hx711_RF_offset))
			print('RF Load Cell zero (Offset) => ' + str(hx711_RF_offset))
			weight_RF.set_offset(hx711_RF_offset)
#
# ------ Update the new offset values in the Offset File
#			
		for load_cells in data['scale']:
			if load_cells['location'] == 'RR':
				load_cells['offset'] = hx711_RR_offset
			elif load_cells['location'] == 'RF':
				load_cells['offset'] = hx711_RF_offset
			elif load_cells['location'] == 'LR':
				load_cells['offset'] = hx711_LR_offset
			elif load_cells['location'] == 'LF':
				load_cells['offset'] = hx711_LF_offset
			else:
				log.write("Error Updating HiveMonitorData.txt!\n")
				print("Error Updating HiveMonitorData.txt!")
				terminal.print("Error Updating HiveMonitorData.txt!")
				
		with open('HiveMonitorData.txt','w') as outfile:
			json.dump(data,outfile)	
			
		print('Zero Calibration Completed!')
		terminal.print('Zero Calibration Completed!')
			
		blynk.virtual_write(19,'0')     # Tell the Blynk App the zero calibration is done
					
#-----------------------------------------------------------
# -- Register Virtual Pins and actions for the Weight Calibrate
# -- Value Blynk Widget (Connected to V20 and V21). 
#-----------------------------------------------------------
@blynk.VIRTUAL_WRITE(20)
def v20_write_handler(value):
	global known_weight
	global known_weight_quarter
	
	known_weight = value[0]
	known_weight_quarter = int(known_weight)/4
	terminal.print('Known Weight set to =>' + str(known_weight))
	
@blynk.VIRTUAL_WRITE(21)
def v21_write_handler(value):
	global known_weight
	global known_weight_quarter
	
	if value[0] == '1':
		log.write("Known Weight Calibrate Button Pressed on Blynk App!\n")
		log.flush()
		print('Known Weight Calibrate Button Pressed on Blynk App')
		terminal.print('Performing a Known Weight Calibration function')
		print('Know Weight => ' + str(known_weight))
		terminal.print('Know Weight => ' + str(known_weight))
		terminal.print('Known Weight Quarter => ' + str(known_weight_quarter))
		
#
# ------ Get New Right Rear Known Weight Reading
#
		if Hive_Weight_RR_Sensor_Installed:
			weight_RR.power_up()	
			measured_weight_RR = (weight_RR.read_average()-weight_RR.get_offset())
			hx711_RR_ratio = int(measured_weight_RR)/int(known_weight_quarter)
			weight_RR.power_down()
			terminal.print('RR Load Cell Weight  => ' + str(measured_weight_RR))
			print('RR Load Cell Weight  => ' + str(measured_weight_RR))
			weight_RR.set_scale(hx711_RR_ratio)
			
#
# ------ Get New Left Rear Known Weight Reading
#
		if Hive_Weight_LR_Sensor_Installed:
			weight_LR.power_up()	
			measured_weight_LR = (weight_LR.read_average()-weight_LR.get_offset())
			hx711_LR_ratio = int(measured_weight_LR)/int(known_weight_quarter)
			weight_LR.power_down()
			terminal.print('LR Load Cell Weight  => ' + str(measured_weight_LR))
			print('LR Load Cell Weight  => ' + str(measured_weight_LR))
			weight_LR.set_scale(hx711_LR_ratio)
			
#
# ------ Get New Left Front Known Weight Reading
#
		if Hive_Weight_LF_Sensor_Installed:
			weight_LF.power_up()	
			measured_weight_LF = (weight_LF.read_average()-weight_LF.get_offset())
			hx711_LF_ratio = int(measured_weight_LF)/int(known_weight_quarter)
			weight_LF.power_down()
			terminal.print('LF Load Cell Weight  => ' + str(measured_weight_LF))
			print('LF Load Cell Weight  => ' + str(measured_weight_LF))
			weight_LF.set_scale(hx711_LF_ratio)
			
#
# ------ Get New Right Front Known Weight Reading
#
		if Hive_Weight_RF_Sensor_Installed:
			weight_RF.power_up()	
			measured_weight_RF = (weight_RF.read_average()-weight_RF.get_offset())
			hx711_RF_ratio = int(measured_weight_RF)/int(known_weight_quarter)
			weight_RF.power_down()
			terminal.print('RF Load Cell Weight  => ' + str(measured_weight_RF))
			print('RF Load Cell Weight  => ' + str(measured_weight_RF))
			weight_RF.set_scale(hx711_RF_ratio)

#
# ------ Update the new ratio values in the Scale Data File
#			
		for load_cells in data['scale']:
			if load_cells['location'] == 'RR':
				load_cells['ratio'] = hx711_RR_ratio
			elif load_cells['location'] == 'RF':
				load_cells['ratio'] = hx711_RF_ratio
			elif load_cells['location'] == 'LR':
				load_cells['ratio'] = hx711_LR_ratio
			elif load_cells['location'] == 'LF':
				load_cells['ratio'] = hx711_LF_ratio
			else:
				log.write("Error Updating HiveMonitorData.txt!\n")
				print("Error Updating HiveMonitorData.txt!")
				terminal.print("Error Updating HiveMonitorData.txt!")
				
		with open('HiveMonitorData.txt','w') as outfile:
			json.dump(data,outfile)	
			
		print('Known Weight Calibration Completed!')
		terminal.print('Known Weight Calibration Completed!')
			
		blynk.virtual_write(21,'0')     # Tell the Blynk App the calibration is done
			
#-----------------------------------------------------------
# -- Define the Blynk reconnect function
#-----------------------------------------------------------
def check_blynk_connection():
	if (blynk.state != 2) and (blynk.state != 1):
		log.write("Blynk is disconnected, trying to connect!\n")
		log.flush()
		print('Blynk is disconnected, Trying to connect!')
		try:
			blynk.connect()
		except:
			return False
		return True
	else:
		return False
	
#-----------------------------------------------------------
# -- Poll and report the sensor readings
#-----------------------------------------------------------	
while True:
	try:
		
		blynk.run()
			
		if time.time() - lastUpdateTime >= updateInterval:

#
# ------ Check the blynk server connection status
#			

			if check_blynk_connection():
				lcd.clear()
				lcd.printlcd(0, 0, 'Reconnected at:')
				lcd.printlcd(0, 1, datetime.datetime.now().strftime("%m/%d %H:%M:%S"))

		
			terminal.print(' ')
			terminal.print('   -----> Time to take some readings! <-----')
			terminal.print(' ')
			terminal.print('Start time => ' + str(datetime.datetime.now().strftime("%H:%M:%S")))
		
#
# ------ Create a new time for the next update
#								

			lastUpdateTime = time.time()			
			
#
# ------ Read Wifi Quality
#

			wifi = read_wifi()
			blynk.virtual_write(8,wifi)
			terminal.print('Wifi Strength = ' + str(wifi) + ' Db')

#
# ------ Get Ambient Temperature and Humidity
#

			if Ambient_Temperature_Sensor_Installed:
			
				humidity, temp = read_DHT22(Humidity_Sensor, DHT22_A_pin)
				
				if temp != None:
					Ambient_Temperature = round(temp * 9/5.0 + 32,1)
					blynk.virtual_write(1,Ambient_Temperature)
					terminal.print('Ambient Temperature => ' + str(Ambient_Temperature) + ' Deg F')
				else:
					log.write("Could not read Ambient Temperature!\n")
					log.flush()
					print('Could not read Ambient Temperature!')
					terminal.print('!!!!!ERROR!!!!! reading Ambient Temperature')
					
				if humidity != None:
					Ambient_Humidity = round(humidity,1)
					blynk.virtual_write(2,Ambient_Humidity)	
					terminal.print('Ambient Humidity => ' + str(Ambient_Humidity) + ' %')
				else:
					log.write("Could not read Ambient Humidity!\n")
					log.flush()
					print('Could not read Ambient Humidity!')
					terminal.print('!!!!!ERROR!!!!! reading Ambient Humidity')	
			
#
# ------ Get Hive Temperature and Humidity
#

			if Hive_Temperature_Sensor_Installed:
			
				humidity, temp = read_DHT22(Humidity_Sensor, DHT22_H_pin)
				DHT22_H_reads = DHT22_H_reads + 1
				
				if temp != None:
					Hive_Temperature = round(temp * 9/5.0 + 32,1)
					blynk.virtual_write(3,Hive_Temperature)
					terminal.print('Hive Temperature => ' + str(Hive_Temperature) + ' Deg F')
				else:
					DHT22_H_fails = DHT22_H_fails + 1
					log.write("Could not read Hive Temperature!\n")
					log.flush()
					print('Could not read Hive Temperature!')
					terminal.print('!!!!!ERROR!!!!! reading Hive Temperature')			
					
				if humidity != None:
					Hive_Humidity = round(humidity,1)
					blynk.virtual_write(4,Hive_Humidity)
					terminal.print('Hive Humidity => ' + str(Hive_Humidity) + ' %')
				else:
					log.write("Could not read Hive Humidity!\n")
					log.flush()
					print('Could not read Hive Humidity!')
					terminal.print('!!!!!ERROR!!!!! reading Hive Humidity')	
					
				DHT22_H_fail_rate = (DHT22_H_fails / DHT22_H_reads) * 100
				blynk.virtual_write(18,DHT22_H_fail_rate)	
				
#
# ------ Get Right Rear Weight Sensor Reading
#
			if Hive_Weight_RR_Sensor_Installed:
				weight_RR.power_up()	
				weight_RR_lbs = weight_RR.get_lbs()
				weight_RR.power_down()
				blynk.run()
				terminal.print('Right Rear Load Cell Weight => ' + str(round(weight_RR_lbs,2)) + ' lbs')
				
#
# ------ Get Left Rear Weight Sensor Reading
#
			if Hive_Weight_LR_Sensor_Installed:
				weight_LR.power_up()	
				weight_LR_lbs = weight_LR.get_lbs()
				weight_LR.power_down()	
				blynk.run()	
				terminal.print('Left Rear Load Cell Weight => ' + str(round(weight_LR_lbs,2)) + ' lbs')		
#
# ------ Get Left Front Weight Sensor Reading
#
			if Hive_Weight_LF_Sensor_Installed:
				weight_LF.power_up()	
				weight_LF_lbs = weight_LF.get_lbs()
				weight_LF.power_down()
				blynk.run()
				terminal.print('Left Front Load Cell Weight => ' + str(round(weight_LF_lbs,2)) + ' lbs')
				
#
# ------ Get Right Front Weight Sensor Reading
#
			if Hive_Weight_RF_Sensor_Installed:
				weight_RF.power_up()	
				weight_RF_lbs = weight_RF.get_lbs()
				weight_RF.power_down()
				blynk.run()
				terminal.print('Right Front Load Cell Weight => ' + str(round(weight_RF_lbs,2)) + ' lbs')
				
#
# ------ Compute and check the total weight
#
				
			weight = round(weight_RR_lbs + weight_LR_lbs + weight_LF_lbs + weight_RF_lbs , 2)
			blynk.virtual_write(5,weight)
			terminal.print('Hive Weight => ' + str(round(weight,2)) + ' lbs')
			
			quarter_weight = weight/4
			err_weight = quarter_weight*.2
			
			if abs(weight_LF_lbs - quarter_weight) > err_weight:
				 terminal.print('!!!!! Hive is not balanced correctly !!!!!')
			elif abs(weight_LR_lbs - quarter_weight) > err_weight:
				 terminal.print('!!!!! Hive is not balanced correctly !!!!!')
			elif abs(weight_RF_lbs - quarter_weight) > err_weight:
				 terminal.print('!!!!! Hive is not balanced correctly !!!!!')
			elif abs(weight_RR_lbs - quarter_weight) > err_weight:
				 terminal.print('!!!!! Hive is not balanced correctly !!!!!')
			else:
				terminal.print('Hive is balanced!')
				
			hivedata = {"weight": weight}
			success = deviceCli.publishEvent("weight", "json", data, qos=0)

#
# ------ Get voltage and power readings
#
			if Power_Sensor_Installed:
				voltage = round(ina.voltage(),2)
				terminal.print('Battery Voltage => ' + str(voltage) + ' Volts')
				if voltage < Shutdown_Voltage:
					print(voltage)
					log.write("Low Voltage, Shutting Down the PI!\n")
					log.flush()
					print('Low Voltatge, Shutting Down the PI')
					terminal.print('!!!!! Low Voltatge Detected !!!!! Shutting Down the PI!')
					time.sleep(5)
					call("sudo shutdown -h now", shell=True)
				power = round(ina.power()/1000,2)
				
				blynk.virtual_write(6,voltage)
				blynk.virtual_write(7,power)
				terminal.print('Power => ' + str(power) + ' Watts')
						
#
# ------ Log the Sensor Readings
#								
			
			LogData(voltage,power,weight,Hive_Humidity,Ambient_Humidity,Hive_Temperature,Ambient_Temperature,wifi)
			
			terminal.print('End time => ' + str(datetime.datetime.now().strftime("%H:%M:%S")))	
			
			
	except KeyboardInterrupt:
		cleanAndExit()
