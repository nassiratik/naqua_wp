#a!/usr/bin/python
#update history
#**************
#tracking in git under project PGORPi
#24/7/18:
#   MAIN : set salinity and temp compensation for DO sensors, write message in status.log. Values are read from config.ini - entries TCOMP and SCOMP
#   Polling : altered to stop flipping oxgen switch with every DO reading. use o2_flag to flip after parsing sensors array
#   devio class: reverse the pin state to reflect relay : LOW sets NO to off and NC to on. Pins are set to High so NO stays on and NC off, to not consume power
#               when in default states - eg alarm off and oxygen off
#
#testing refresh.sh

import io, requests         # used to create file streams
import commands
import RPi.GPIO as GPIO
import fcntl      # used to access I2C parameters like addresses
import os
import glob
import time       # used for sleep delay and timestamps
import datetime
from time import strftime
import urllib
import string     # helps parse strings
#import Adafruit_CharLCD as LCD
import serial
import smbus
from sys import argv

#LCD I2C constants
LCD_ATTACHED = False
LCD_I2C_ADDR  = 0x27 # I2C device address
LCD_WIDTH = 16   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line

LCD_BACKLIGHT  = 0x08  # On
LCD_NOBL = 0x00  # Off
LCD_YESBL = 0x08  # On
LCD_DEFBL = 0x08  # On

ENABLE = 0b00000100 # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

config={}
sensors=[]
DEBUGMODE = True
emptyOfflineData = True
internetOnline = True
offlineCtr=0
oxygen_is_running=False
localpath='/home/pi/PGO/v1_1/'
# Define LCD column and row size for 16x2 LCD.
lcd_columns = 16
lcd_rows    = 2
lcd_DO = lcd_temp = "0.0"
caprobes=object()
pincontrol = object()
lcd1=object()
bus=object()
enable_interactive=False
interactive_flag = False
intmode_button = 0.0
# Temp variables
device_file = ""
ValidReadings={
    "OxygenMin":3.0,
    "OxygenMax":25.0,
    "SalinityMin":10.0,
    "SalinityMax":50.0,
    "WaterMin":10,
    "WaterMax":350,
    "TempMin":15.0,
    "TempMax":50.0,
    "pHMin":0.0,
    "pHMax":15.0
    }


def lcd_init():
    # Initialise display
    lcd_byte(0x33,LCD_CMD) # 110011 Initialise
    lcd_byte(0x32,LCD_CMD) # 110010 Initialise
    lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
    lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
    lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
    lcd_byte(0x01,LCD_CMD) # 000001 Clear display
    time.sleep(E_DELAY)

def lcd_byte(bits, mode):
    # Send byte to data pins
    # bits = the data
    # mode = 1 for data
    #        0 for command
    global bus
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

    # High bits
    bus.write_byte(LCD_I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)

    # Low bits
    bus.write_byte(LCD_I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    # Toggle enable
    global bus
    time.sleep(E_DELAY)
    bus.write_byte(LCD_I2C_ADDR, (bits | ENABLE))
    time.sleep(E_PULSE)
    bus.write_byte(LCD_I2C_ADDR,(bits & ~ENABLE))
    time.sleep(E_DELAY)

def lcd_string(message,line,bl=LCD_DEFBL):
  # Send string to display
    if LCD_ATTACHED:
        global LCD_BACKLIGHT
        LCD_BACKLIGHT = bl
        message = message.ljust(LCD_WIDTH," ")

        lcd_byte(line, LCD_CMD)

        for i in range(LCD_WIDTH):
            lcd_byte(ord(message[i]),LCD_CHR)
    


class AtlasI2C:
    long_timeout = 1.5         	# the timeout needed to query readings and calibrations
    short_timeout = .5         	# timeout for regular commands
    default_bus = 1         	# the default bus for I2C on the newer Raspberry Pis, certain older boards use bus 0
    default_address = 98     	# the default address for the sensor
    current_addr = default_address

    def __init__(self, address=default_address, bus=default_bus):
        # open two file streams, one for reading and one for writing
        # the specific I2C channel is selected with bus
        # it is usually 1, except for older revisions where its 0
        # wb and rb indicate binary read and write
        self.file_read = io.open("/dev/i2c-"+str(bus), "rb", buffering=0)
        self.file_write = io.open("/dev/i2c-"+str(bus), "wb", buffering=0)

        # initializes I2C to either a user specified or default address
        self.set_i2c_address(address)

    def set_i2c_address(self, addr):
        # set the I2C communications to the slave specified by the address
        # The commands for I2C dev using the ioctl functions are specified in
        # the i2c-dev.h file from i2c-tools
        I2C_SLAVE = 0x703
        fcntl.ioctl(self.file_read, I2C_SLAVE, addr)
        fcntl.ioctl(self.file_write, I2C_SLAVE, addr)
        self.current_addr = addr
        print("i2c address: " + str(addr))

    def write(self, cmd):
        # appends the null character and sends the string over I2C
        cmd += "\00"
        try:
            self.file_write.write(cmd)
        except:
            pass

    def read(self, num_of_bytes=31):
        # reads a specified number of bytes from I2C, then parses and displays the result
        try:
            res = self.file_read.read(num_of_bytes)         # read from the board
        except:
            res = '0'
        response = filter(lambda x: x != '\x00', res)     # remove the null characters to get the response
        if ord(response[0]) == 1:             # if the response isn't an error
            # change MSB to 0 for all received characters except the first and get a list of characters
            char_list = map(lambda x: chr(ord(x) & ~0x80), list(response[1:]))
            # NOTE: having to change the MSB to 0 is a glitch in the raspberry pi, and you shouldn't have to do this!
            return "Recvd Data " + ''.join(char_list)     # convert the char list to a string and returns it
        else:
            return "Error " + str(ord(response[0]))

    def query(self, string):
        # write a command to the board, wait the correct timeout, and read the response
        self.write(string)

        # the read and calibration commands require a longer timeout
        if((string.upper().startswith("R")) or
            (string.upper().startswith("CAL"))):
            time.sleep(self.long_timeout)
        elif string.upper().startswith("SLEEP"):
            return "sleep mode"
        else:
            time.sleep(self.short_timeout)

        return self.read()

    def close(self):
        self.file_read.close()
        self.file_write.close()    

    def list_i2c_devices(self):
        prev_addr = self.current_addr # save the current address so we can restore it after
        i2c_devices = []
        for i in range (0,128):
            try:
                self.set_i2c_address(i)
                self.read()
                i2c_devices.append(i)
            except IOError:
                pass
        self.set_i2c_address(prev_addr) # restore the address we were using
        return i2c_devices

class devio:

    def __init__(self):

        self.PIN_link = 5 #5 link status. high = disconnected
        self.PIN_op = 6 #6 device operation indicator. high=script is running
        self.PIN_alarm=13 #22 alarm indicator. high=an alarm is active
#        self.PIN_oxygenstate = 35 # 19oxygen supply indicator : high=oxygen solenoid valve is open
#        self.PIN_oxygenswitch = 37 #26 oxygen switch. set high to open oxygen supply solenoid valve
        self.PIN_motorstate = 19 #this pin is now being used as actuator indicator in motor class
        self.PIN_oxygenswitch = 26 #21 oxygen switch. set high to open oxygen supply solenoid valve

        self.PIN_link_curr = 0 #link status. high = disconnected
        self.PIN_op_curr = 0 #device operation indicator. high=script is running
        self.PIN_alarm_curr=0 #alarm indicator. high=an alarm is active
        self.PIN_oxygen_curr = 0 #oxygen supply indicator : high=oxygen solenoid valve is open
        self.PIN_motor_curr = 0 #motor operation led

        self.pinHigh=1
        self.pinLow=0


        GPIO.setmode(GPIO.BCM)
#        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.PIN_link,GPIO.OUT) 
        GPIO.setup(self.PIN_op ,GPIO.OUT) 
        GPIO.setup(self.PIN_alarm,GPIO.OUT) 
        GPIO.setup(self.PIN_motorstate,GPIO.OUT) 
        GPIO.setup(self.PIN_oxygenswitch,GPIO.OUT) 
        GPIO.output(self.PIN_op,GPIO.HIGH)
        self.PIN_op_curr=self.pinHigh
        GPIO.output(self.PIN_oxygenswitch,GPIO.HIGH) #make sure oxygen solenoid valve is closed

    def resetall(self):
        self.set_op(self.pinLow)
        self.set_link(self.pinLow)
        self.set_motorstate(self.pinLow)
        self.set_alarm(self.pinHigh)
        self.flip_oxygen(self.pinHigh)
        

    def set_op(self,op_state):
        if op_state == self.pinHigh:
            GPIO.output(self.PIN_op,GPIO.LOW)
            self.PIN_op_curr = self.pinHigh
        else:
            GPIO.output(self.PIN_op,GPIO.HIGH)
            self.PIN_op_curr=0

    def set_link(self,link_state):
        if link_state == self.pinHigh:
            GPIO.output(self.PIN_link,GPIO.HIGH)
            self.PIN_link_curr=self.pinHigh
        else:
            GPIO.output(self.PIN_link,GPIO.LOW)
            self.PIN_link_curr=0

    def set_alarm(self,alarm_state):
        if alarm_state == self.pinHigh:
            GPIO.output(self.PIN_alarm,GPIO.LOW)
            self.PIN_alarm_curr=self.pinHigh
        else:
            GPIO.output(self.PIN_alarm,GPIO.HIGH)
            self.PIN_alarm_curr=0

    def set_motorstate(self,motor_state):
        GPIO.output(self.PIN_motorstate,motor_state)
        self.PIN_motor_curr=motor_state

    def flip_oxygen(self,on_off):
        if on_off == self.pinHigh:
            GPIO.output(self.PIN_oxygenswitch,GPIO.LOW)
#            self.set_oxygenstate(self.pinHigh)
        else:
            GPIO.output(self.PIN_oxygenswitch,GPIO.HIGH)
#            self.set_oxygenstate(self.pinLow)
    def flash(self,event):
        curr=self.PIN_op_curr
        motor_curr = self.PIN_motor_curr
        if event == 'goodread':
            self.set_op(self.pinHigh)
            time.sleep(0.4)
            self.set_op(self.pinLow)
            time.sleep(0.2)
            
        elif event == 'upload':
            self.set_link(self.pinHigh)
            time.sleep(0.2)
            self.set_link(self.pinLow)
        elif event == 'badread':
            for i in range(4):
                self.set_op(self.pinHigh)
                time.sleep(0.3)
                self.set_op(self.pinLow)
                time.sleep(0.2)   
        elif event == 'EnterIMode':
                print("Flashing EnterIMode")
                self.set_motorstate(self.pinHigh)
                time.sleep(2)
                self.set_motorstate(self.pinLow)
                self.set_motorstate(motor_curr)
        elif event == 'ExitIMode':
            for i in range(3):
                self.set_motorstate(self.pinHigh)
                time.sleep(0.5)
                self.set_motorstate(self.pinLow)
                time.sleep(0.2)   
            self.set_motorstate(motor_curr)
        elif event == 'IModeError':
            print("Flashing IModeError")
            for i in range(4):
                self.set_motorstate(self.pinHigh)
                time.sleep(0.3)
                self.set_motorstate(self.pinLow)
                time.sleep(0.2)   
            self.set_motorstate(motor_curr)
        self.set_op(curr)
            
    
    def get_op(self):
        return 'On' if self.PIN_op_curr == self.pinHigh else 'Off'

    def get_link(self):
        return 'On' if self.PIN_link_curr == self.pinHigh else 'Off'

    def get_alarm(self):
        return 'On' if self.PIN_alarm_curr == self.pinHigh else 'Off'

    def get_oxygenstate(self):
        return 'On' if self.PIN_oxygen_curr == self.pinHigh else 'Off'


class motor:
    probe_posn = ''

    def __init__(self, motorenable, runsecs=5):

        self.PIN_relay1 = 16 #Motor Relay no.1
        self.PIN_relay2 = 20 #Motor Relay no.1
        self.PIN_motormain = 25 #Motor main power relay switch
#        self.PIN_water = 21 #water sensor
#        self.PIN_maxlevel = 12 #probe level sensor switch
        self.PIN_led = 19 #use GPIO19 for led indicating motor is active. this pin was initially for O2-state
        self.PIN_maxlevel = 23 #12 probe level sensor switch
        self.PIN_lowlevel = 24 #probe lowest  level. HIGH means probes have reached lower limit and motor to be switched off
        self.waterstate = False
        self.arduino=""
        
        self.m_down = 5
        self.m_up=5
        self.m_settle = int(config['SETTLE'])
        self.m_runsecs=runsecs
        self.m_enable = True if (motorenable == 'yes') else False        

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.PIN_relay1,GPIO.OUT) 
        GPIO.setup(self.PIN_motormain,GPIO.OUT)
        GPIO.setup(self.PIN_relay2,GPIO.OUT) 
        GPIO.setup(self.PIN_led,GPIO.OUT) 
        GPIO.setup(self.PIN_maxlevel,GPIO.IN) 
        GPIO.output(self.PIN_motormain,GPIO.HIGH) #make sure main motor power relay is on closed state=> powered off
        GPIO.output(self.PIN_relay1,GPIO.HIGH) #make sure main motor power relay is on closed state=> powered off
        GPIO.output(self.PIN_relay2,GPIO.HIGH) #make sure main motor power relay is on closed state=> powered off
        #self.waterstate = GPIO.input(self.PIN_water)
        self.probe_posn = 'up' #self.raise_probes()
        print('Probe Position during class init = ' + self.probe_posn)
    def setarduino(self,objard):
        self.arduino=objard
        try:
            res = "In Water" if self.ardReadWD() else "Out of water"
        except:
            res = "Arduino not detected!"
        print('Arduino: Initial probe position: ' + res )

    def ardReadWD(self):
        probeInWater = False
        try:
                self.arduino.write('1') #read water detector
        except:
                return probeInWater 
        dataexists=True
        ardtext=""
        while dataexists:
            txt1=self.arduino.readline()
            if (len(txt1) > 3) : ardtext = txt1
            if (len(txt1) < 5) : dataexists = False
        if (len(ardtext)>5):
            if (string.split(ardtext,",")[0] == 'WD'):
                    probeInWater = True if (string.split(ardtext,",")[1] == 'YES') else False
                    WD_Reading = string.split(ardtext,",")[2]
 
        print('Arduino: Water detector (Motors Class): ' + ardtext )

        return probeInWater

    def lower_probesX(self,runsecs=0):
        if (not self.m_enable) : return "down"
        if ((self.probe_posn == 'jam') or (self.probe_posn == 'down')) : return self.probe_posn
        GPIO.output(self.PIN_led,GPIO.HIGH)
        wtr_state=GPIO.LOW #GPIO.input(self.PIN_water)
        GPIO.output(self.PIN_motormain,GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.PIN_relay1,GPIO.HIGH)
        GPIO.output(self.PIN_relay2,GPIO.HIGH)
        time.sleep(1) #one second delay to ensure two relays are properly set. this is to avoid shorting
        GPIO.output(self.PIN_motormain,GPIO.LOW)
        print('motor switched on to lower probes')
        
        if (runsecs == 0):
            time.sleep(self.m_runsecs)
        else:
            time.sleep(runsecs)
            
        #loop to check if surface of water reached
        wtr_ctr=0
        #while ((wtr_state == 0) and (wtr_ctr < 20)):
        #        time.sleep(0.5) #read water sensor every half second
        #        wtr_ctr = wtr_ctr + 1
        #        wtr_state=GPIO.input(self.PIN_water)
        #        if (wtr_state == 1):
        #                time.sleep(0.01) #debounce water sensor
        #                wtr_state=GPIO.input(self.PIN_water)
                        
        GPIO.output(self.PIN_motormain,GPIO.HIGH) #power off
        print('motor switched off - lower probes')
        GPIO.output(self.PIN_led,GPIO.LOW)
        if (wtr_ctr > 19):
                self.probe_posn = "jam"
                return "jam"
        else:
                time.sleep(self.m_settle)
                self.probe_posn = "down"
                return "down"
    
    def lower_probesX1(self,runsecs=0):
        if (not self.m_enable) : return "down"
        if ((self.probe_posn == 'jam') or (self.probe_posn == 'down')) : return self.probe_posn
        GPIO.output(self.PIN_led,GPIO.HIGH)
        level_state=GPIO.input(self.PIN_lowlevel) #GPIO.input(self.PIN_water)
        GPIO.output(self.PIN_motormain,GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.PIN_relay1,GPIO.HIGH)
        GPIO.output(self.PIN_relay2,GPIO.HIGH)
        time.sleep(1) #one second delay to ensure two relays are properly set. this is to avoid shorting
        GPIO.output(self.PIN_motormain,GPIO.LOW)
        print('motor switched on to lower probes')
        ht_ctr=0
        if (runsecs == 0):
            while ((level_state == 0) and (ht_ctr < self.m_runsecs*8)):
                    time.sleep(0.25) #read water sensor every half second
                    ht_ctr = ht_ctr + 1
                    level_state=GPIO.input(self.PIN_lowlevel)
                    if (level_state == 1):
                            #print('level_state has changed')
                            time.sleep(0.01) #debounce water sensor
                            level_state=GPIO.input(self.PIN_lowlevel)
                            #if (level_state == 1) : print('level_state change confirmed')
        else:
            time.sleep(runsecs)
            
                       
        GPIO.output(self.PIN_motormain,GPIO.HIGH) #power off
        print('motor switched off - lower probes')
        GPIO.output(self.PIN_led,GPIO.LOW)
        if (ht_ctr > (self.m_runsecs*8-1)):
                self.probe_posn = "jam"
                return "jam"
        else:
                time.sleep(self.m_settle)
                self.probe_posn = "down"
                return "down"
            
    def halt_probes(self):
        GPIO.output(self.PIN_motormain,GPIO.HIGH)
        GPIO.output(self.PIN_led,GPIO.LOW)
        self.probe_posn = "mid"
        print("Actuator Stopped midway")
        return "mid"
        
        
    def lower_probes(self,runsecs=0):
        if (not self.m_enable) : return "down"
        if ((self.probe_posn == 'jam') or (self.probe_posn == 'down')) : return self.probe_posn
        GPIO.output(self.PIN_led,GPIO.HIGH)
        GPIO.output(self.PIN_motormain,GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.PIN_relay1,GPIO.HIGH)
        GPIO.output(self.PIN_relay2,GPIO.HIGH)
        time.sleep(1) #one second delay to ensure two relays are properly set. this is to avoid shorting
        GPIO.output(self.PIN_motormain,GPIO.LOW)
        print('motor switched on to lower probes')
        ht_ctr=0
        if (runsecs == 0):
                time.sleep(self.m_runsecs)
        else:
                time.sleep(runsecs)
                       
        GPIO.output(self.PIN_motormain,GPIO.HIGH) #power off
        print('motor switched off - lower probes')
        GPIO.output(self.PIN_led,GPIO.LOW)
#        if (GPIO.input(self.PIN_lowlevel) == 0): 
#                self.probe_posn = "jam"
#                return "jam"
#        else:
        time.sleep(self.m_settle)
        if runsecs==0:
            self.probe_posn = "down"
        else:
            self.probe_posn = "manual"
        return self.probe_posn
        
    def raise_probes(self,runsecs=0):
        if (not self.m_enable) : return "up"
        if ((self.probe_posn == 'jam') or (self.probe_posn == 'up')) : return self.probe_posn
        GPIO.output(self.PIN_led,GPIO.HIGH)
        GPIO.output(self.PIN_motormain,GPIO.HIGH)
        GPIO.output(self.PIN_relay1,GPIO.LOW)
        GPIO.output(self.PIN_relay2,GPIO.LOW)
        time.sleep(1) #one second delay to ensure two relays are properly set. this is to avoid shorting
        GPIO.output(self.PIN_motormain,GPIO.LOW)
        print('motor switched on to raise probes')
        
        if (runsecs == 0):
                time.sleep(self.m_runsecs)
        else:
            time.sleep(runsecs)
            
        GPIO.output(self.PIN_motormain,GPIO.HIGH) #power off
        time.sleep(0.5)
        GPIO.output(self.PIN_relay1,GPIO.HIGH)
        GPIO.output(self.PIN_relay2,GPIO.HIGH)
        GPIO.output(self.PIN_led,GPIO.LOW)
        print('motor switched off -raise probes')
#        if (GPIO.input(self.PIN_maxlevel) == 0):
#                self.probe_posn = "jam"
#                return "jam"
#        else:
        if runsecs==0:
            self.probe_posn = "up"
        else:
            self.probe_posn = "manual"
        return self.probe_posn


    

    def raise_probesX(self,runsecs=0):
        if (not self.m_enable) : return "up"
        if ((self.probe_posn == 'jam') or (self.probe_posn == 'up')) : return self.probe_posn
        GPIO.output(self.PIN_led,GPIO.HIGH)
        level_state=GPIO.input(self.PIN_maxlevel)
        GPIO.output(self.PIN_motormain,GPIO.HIGH)
        GPIO.output(self.PIN_relay1,GPIO.LOW)
        GPIO.output(self.PIN_relay2,GPIO.LOW)
        time.sleep(1) #one second delay to ensure two relays are properly set. this is to avoid shorting
        GPIO.output(self.PIN_motormain,GPIO.LOW)
        print('motor switched on to raise probes')
        #loop to check if surface of max height reached
        #if (runsecs == 0):
        #    time.sleep(self.m_runsecs)
        #else:
        #    time.sleep(runsecs)
        ht_ctr=0
        
        if (runsecs == 0):
            while ((level_state == 0) and (ht_ctr < self.m_runsecs*8)):
                    time.sleep(0.25) #read water sensor every half second
                    ht_ctr = ht_ctr + 1
                    level_state=GPIO.input(self.PIN_maxlevel)
                    if (level_state == 1):
                            #print('level_state has changed')
                            time.sleep(0.01) #debounce water sensor
                            level_state=GPIO.input(self.PIN_maxlevel)
                            #if (level_state == 1) : print('level_state change confirmed')
        else:
            time.sleep(runsecs)
            
        GPIO.output(self.PIN_motormain,GPIO.HIGH) #power off
        time.sleep(0.5)
        GPIO.output(self.PIN_relay1,GPIO.HIGH)
        GPIO.output(self.PIN_relay2,GPIO.HIGH)
        GPIO.output(self.PIN_led,GPIO.LOW)
        print('motor switched off -raise probes')
        if (ht_ctr > (self.m_runsecs*8-1)):
                self.probe_posn = "jam"
                return "jam"
        else:
                self.probe_posn = "up"
                return "up"
    def sysEnabled(self, yn):
        self.m_enable = True if (yn == 'yes') else False
        if self.m_enable :
            print('gantry is enabled')
            for i in range(3):
                    GPIO.output(self.PIN_led,GPIO.HIGH)
                    time.sleep(1)
                    GPIO.output(self.PIN_led,GPIO.LOW)
                    time.sleep(0.2)
            self.raise_probes()
        else:
            for i in range(4):
                    GPIO.output(self.PIN_led,GPIO.HIGH)
                    time.sleep(0.3)
                    GPIO.output(self.PIN_led,GPIO.LOW)
                    time.sleep(0.2)
            print('gantry is disabled')

    def getSysStatus(self):
        return self.m_enable

    def getProbePosition(self):
        return self.probe_posn
    
    def resetProbePosition(self):
        self.probe_posn = ''


    def get_position(self):
        return self.probe_posn





def readConfig():
    global sensors
    cfg = open(localpath+"pgoconfig.ini")
    for line in cfg:
        pstr=line
        ppos = pstr.upper().find("=")
        pname = pstr.upper()[:ppos].strip()
        pval = pstr[ppos+1:].strip()
        if not (pname == "SENSOR"):
            config[pname]=pval
        else:
            #add installed sensor
            arrS=pval.split(',')
            sensor={'SensorType':arrS[0], 'SensorID':arrS[1], 'Comm':arrS[2], 
            'Addr':arrS[3], 'Desc':arrS[4], 'File':'NA', 
            'Reading':'0', 'Error':0, 'ErrorLog':0}
            sensors.append(sensor)
            print(arrS)
    cfg.close()
def init1Wire():
    # Initialize the DS18B20.
    os.system('modprobe w1-gpio')
    os.system('modprobe w1-therm')
    base_dir = '/sys/bus/w1/devices/'
    device_folders = glob.glob(base_dir + '28*')
    if len(device_folders) > 0:
        for folder in device_folders:
            print(folder)
            sno_pos=folder.find('/28')
            if sno_pos != -1:
                sno = folder[sno_pos+1:]
            else:
                sno=''
            for sensorinfo in sensors:
                if sensorinfo["Addr"].upper() == sno.upper():
                    sensorinfo["File"]=folder+'/w1_slave'

    if DEBUGMODE:
        for sensor in sensors:
            print('init1wire: ' + sensor['SensorType'] + '; SensorID: ' + sensor['SensorID'] + '; Desc: ' + sensor['Desc'] + '; Addr: ' + sensor['Addr'] + '; File: ' + sensor['File'])

        print("Temp Sensors found=" + str(len(device_folders)))

def read_temp_raw(devfile):
            f = open(devfile, 'r')
            lines = f.readlines()
            f.close()
            return lines

def read_temp(devfile):
    lines = read_temp_raw(devfile)
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw(devfile)
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        # temp_f = temp_c * 9.0 / 5.0 + 32.0
        return str(temp_c)
def setOfflineDataFlag():
    global emptyofflineData
    try:
        offline_file=open(localpath+"OfflineData.txt")
    except IOError:
        emptyofflineData = True
        print("Warning: Offline Data File not initialized")
        return emptyofflineData
    str = offline_file.read()
    offline_file.close()
    emptyofflineData = True if (len(str)==0) else False
    if DEBUGMODE:
            print("No Offline Data" if emptyofflineData else "Offline Data Exists")
    return emptyofflineData

def write_offline(sensortype, payload):
    #write data to local file
    global emptyofflineData
    if DEBUGMODE:
        print("writing data to local file")
    strpayload = str(payload['Controller']) + "|" + sensortype + "|" + str(payload['Sensor']) + "|" + payload['ReadingTime'] + "|"  
    strval = ''
    for x in payload['value']:
        if isinstance(payload['value'],dict):
            if not (str(x) == 'default'):
                strval += (str(payload['value'][x]) + ',')
        else:
            if not (str(x) == 'default'):
                strval += (str(x) + ',')
    strval = strval[0:len(strval)-1]
    strpayload += (strval + "|" + str(payload['Errors']))

    offline_file=open(localpath+"OfflineData.txt",'a')
    offline_file.write(strpayload + '\n')
    offline_file.close()
    emptyofflineData = False

def log_errors(pincontrol):
    #scan sensor list and log reported errors
    error_log=open(localpath+"errors.log",'a')
    dateWrite=time.strftime("%Y-%m-%d")
    timeWrite=time.strftime("%H:%M:%S")
    errctr=0
    for sensor in sensors:
        if sensor['Error'] > sensor['ErrorLog']:
            #new error has been encountered
            err_msg = 'Sensor Read Error : SensorType=' + sensor['SensorType']
            err_msg += '; SensorID=' + sensor['SensorID']
            err_msg += ' : ' + sensor['Desc']
            err_msg += '; Date=' + dateWrite
            err_msg += '; Time=' + timeWrite
            error_log.write(err_msg + '\n')
            sensor['ErrorLog'] = sensor['Error']
            if sensor['Error'] > int(config['ERRORTHRESHOLD']) and internetOnline:
                result = upload_error_msg(sensor)
                sensor['Error'] = 0
                sensor['ErrorLog'] = 0
                errctr+=1
                
    error_log.close()
    on_off = 1 if errctr > 0 else 0
    pincontrol.set_alarm(on_off) #turn on alarm LED on or off depending on existence of errors. ( alarm strobe, siren, etc, if installed)
        

def log_status(status):
    #scan sensor list and log reported errors
    status_log=open(localpath+"status.log",'a')
    dateWrite=time.strftime("%Y-%m-%d")
    timeWrite=time.strftime("%H:%M:%S")
    err_msg = status
    err_msg += '****'
    err_msg += '; Date=' + dateWrite
    err_msg += '; Time=' + timeWrite
    status_log.write(err_msg + '\n')
                
    status_log.close()


def upload_error_msg(sensor):
    #upload an error message to webapp after exceeding threshold
    ctr=0
    if DEBUGMODE:
        print("uploading error log")
    dateWrite=time.strftime("%Y-%m-%d %H:%M")
    #timeWrite=time.strftime("%H:%M:%S")

    payload = {
        'controller=' : config['CONTROLLER'],
        'Sensor' : sensor['SensorID'],
        'error' : 'sensor read error',
        'time=' : dateWrite,
    }    
    

    try:
        response = requests.post(config["SERVERURL"]+'/sensor_error',json=payload)
        return "OK" if  response.status_code == 200 else 'FAIL, Response = ' + str(response.status_code)
    except:
        return "FAIL, Not able to connect"

def upload_offline_data():
#when website is back online, upload buffered data and empty offline file
    global emptyofflineData
    ctr=0
    if DEBUGMODE:
        print("uploading offline data")
    offline_file=open(localpath+"OfflineData.txt")
    for payload in offline_file:
#        try:
            response = requests.post(config["SERVERURL"]+'/sensor_reading',json=parse_payload(payload),timeout=3)
            if response.status_code == 201:
                ctr += 1
                print("Offline ctr = " + str(ctr))
            else:
                print("fail after trying, response " + str(response.status_code))
                offline_file.close()
                return "FAIL, response=" + str(response.status_code)
 #       except:
            # print("fail with exception")
            # offline_file.close()
            # return "FAIL" + str(ctr)
    offline_file.close()
    # offline_file=open(localpath+"OfflineData.txt","w")
    # offline_file.truncate()
    # offline_file.close()
    emptyofflineData = True
    return "PASS"+str(ctr) 

def data_is_good(data):
    if isinstance(data,dict):
        for x in data:
            if data[x] == 'bad':
                return False
    else:
        if data[0] == 'bad': return False
    return True


def upload_data(pincontrol):
    global oxygen_is_running
    commStatus = 1  #no exception
    fail_ctr = bad_data_ctr = success_ctr = redirected_ctr = 0

    dateWrite=time.strftime("%Y-%m-%d %H:%M")
    timeWrite=time.strftime("%H:%M:%S")

    pincontrol.flash('upload')
    o2_flag=False
    for sensor in sensors:
        payload = {'Controller':config["CONTROLLER"],
            'Sensor':sensor['SensorID'],
            'ReadingTime':dateWrite,
            'value':sensor['Reading'],
            'Errors': sensor['Error'],
            }
#        payload = parse_payload(config["CONTROLLER"] + ':' + sensor['SensorID'] + ':' + dateWrite + ':' + perse_reading(sensor['SensorID'],sensor['Reading']) + ':' + sensor['Error'] )

        if  ((sensor['Error'] == 0) and data_is_good(sensor['Reading'])):   #no read error
            if not internetOnline:
                if DEBUGMODE:
                    print("offline - no connection")
                write_offline(payload)
                redirected_data_ctr += 1
                commStatus = 2 #no attempt to access server, until internetOnline is True. To save time
                print("server not accessible")
            else:
                try:
                    response = requests.post(config["SERVERURL"]+'/sensor_reading',json=payload,timeout=3)
                    link_flag = response.status_code
                    if link_flag == 201: #ok
                        success_ctr += 1
                        if emptyofflineData == False:
                            log_status("Connection is restored, uploading offline data")
                            upl=upload_offline_data()
                            bad_data_ctr=0
                        commStatus = 1 #success
                        pincontrol.set_link(0) #no exception
                    else:
                        commStatus = 0 #fail
                except:
                    print("exception while attempting to upload")
                    link_flag = 999 #website is offline or URL is incorrect
                if not link_flag == 201:             
                    if DEBUGMODE:
                        print("offline, exception " + str(link_flag))
                    log_status("Connection is lost, saving data to local file")
                    print(payload)
                    write_offline(sensor['SensorType'],payload)
                    fail_ctr += 1
                    commStatus = 0
                    pincontrol.set_link(1) #turn LED on to indicate disconnected state
            if sensor['SensorType'] == '20' and data_is_good(sensor['Reading']):
                #print("checking DO level...")
                try:
                  if float(sensor['Reading']['default']) < float(config['MIN_OXYGEN']):
                      o2_sid = sensor['SensorID']
                      o2_reading=sensor['Reading']['default']
                      o2_flag=True
                except:
                  pass
        else:
            str1=''
            if isinstance(payload,dict):
                # for x in payload:
                #     str1 = str1 + x + ' : ' + payload[x] + ';'
                pass
            else:
                for x in payload:
                    str1 = str1 + x + '; ' 
            print("bad data on sensor : " + sensor['SensorID'])
            print(payload)
            bad_data_ctr += 1

    if o2_flag:
        if pincontrol.get_oxygenstate()=="Off":
            pincontrol.flip_oxygen(1)
            log_status('DO level below minimum on sensor ' + o2_sid + '. Reading is ' + o2_reading+ ', min is ' + config['MIN_OXYGEN'] + ' - Oxygen switched on')
    else:
        if pincontrol.get_oxygenstate()=="On":
            pincontrol.flip_oxygen(0)
            log_status('DO level above minimum on all sensors, min is ' + config['MIN_OXYGEN'] + ' - Oxygen switched off')
                            
    return [success_ctr, fail_ctr, redirected_ctr, bad_data_ctr]

def readWL(arduino,sensor):
    WL_Reading = '0.0'
    conv_factor = 1
    try:
      arduino.write('2' if sensor=='sonar' else '3') #read water level detector
      dataexists=True
      while dataexists:
        txt1=arduino.readline()
        if (len(txt1) > 3) : ardtext = txt1
        if (len(txt1) < 5) : dataexists = False
      if sensor == 'sonar':
        if (len(ardtext)>5):
          if (string.split(ardtext,",")[0] == 'WL'):
            WL_Reading = string.split(ardtext,",")[2]

          ht=int(config['SONARHT'])
          WL_Reading  = string.split(WL_Reading,".")[0]
          rd = int(WL_Reading) #this is to truncate the decimal part whch contains \r\n
          if not (ValidReadings["WaterMin"] <= rd <= ValidReadings["WaterMax"]): rd=0
          if (rd > 0): rd = ht - rd
        else:
          rd = 888
          
      elif sensor == 'pressure':

        if (len(ardtext)>5):
          if (string.split(ardtext,",")[0] == 'WP'):
            WL_Reading = string.split(ardtext,",")[2]
          rd = int(string.split(WL_Reading,".")[0]) #this is to truncate the decimal part whch contains \r\n
          if (rd > 0): rd = rd*conv_factor
          print("Pressure reading.."+ WL_Reading)
        else:
          rd = 888

      else:
        rd = 999
      if rd == 888:
        return 'NoData'
      elif rd == 999:
        return 'sensor!'
      else:
        return  str(rd) 
    except:
      return  'Error!'

def parse_payload(strpayload):
    """reformats text into dict of 'reading option':value. Input is separated by ':', with readings as csv. 
    output is dict, with readings as nested dict in case of stype 20 and 40
    """
    # try:
    #     Sensor_Options=config['ECSENSOROPTIONS'].upper().split(',') if sensortype == 40 else config['DOSENSOROPTIONS'].upper().split(',')
    # except:
    #     Sensor_Options=['TDS','EC','S'] if sensortype == 40 else ['MG','PS']
    pl = strpayload.split('|')
    
    payload = {'Controller': int(pl[0]), 'Sensor': int(pl[2]), 'ReadingTime':pl[3],'value':perse_reading(int(pl[1]),pl[4].split(',')), 'Errors':int(pl[5])}
    print(payload)

    return payload

def perse_reading(sensortype,reading):
    #reformats csv text into dict of 'reading option':value 
    if sensortype in (20,40):
        try:
            Sensor_Options=config['ECSENSOROPTIONS'].upper().split(',') if sensortype == 40 else config['DOSENSOROPTIONS'].upper().split(',')
        except:
            Sensor_Options=['TDS','EC','S'] if sensortype == 40 else ['MG','PS']
        if not (len(Sensor_Options) == len(reading)) :
            print(Sensor_Options)
            print(reading)
            i = 0
            rdg=[]
            while i < len(Sensor_Options):
                rdg.append('bad')
                i += 1
        else:
            rdg = reading
        rdg_out = {}
        i = 0
        while i < len(Sensor_Options):
            rdg_out[Sensor_Options[i]] = rdg[i]
            i += 1
        rdg_out["default"] = rdg[Sensor_Options.index('S')] if sensortype == 40 else rdg[Sensor_Options.index('MG')]
    else:
        rdg_out = reading 
    return rdg_out

def pollDevice(device, lcd1, delaytime, pincontrol, probes, arduino):
    global offlineCtr, internetOnline, sensors, T_Compensation, S_Compensation, lcd_DO, lcd_temp
    lcd_temp = lcd_DO ='0.0'
    lcd_EC=lcd_pH=WL_Reading=WP_Reading='0.0'
    EC_Has_Changed = False
    Temp_Has_Changed = False
    conv_factor=1 #convert water pressure to depth
    dline=''
#clear previous readings, reset error flags
    for sensor in sensors:
        sensor['Reading'] = 'err'
    lcd_string("Reading...",LCD_LINE_1)
    lcd_string("",LCD_LINE_2)
    probe_result = probes.lower_probes()
    if DEBUGMODE: print('probe position before reading : ' + probes.getProbePosition())
#read all 1-wire temperature sensors
        
    if (probes.getProbePosition() == 'down' or not probes.getSysStatus()):
            lcd_temp='0.0'
            lcd_DO='0.0'
            lcd_pH='0.0'
            lcd_EC='0.0'
            WL_Reading='0.0'
            WL_Peading='0.0'
            
        
            for sensor in sensors:
                if (sensor['SensorType'] in ('10','90') and len(sensor['File']) > 2):
                    try:
                        tr = read_temp(sensor['File'])
                        if not (ValidReadings["TempMin"] <= float(tr) <= ValidReadings["TempMax"]): tr='0.0'
                        sensor['Reading'] = [tr]
                        sensor['Error'] = 0
                        if sensor['SensorType'] == '10':
                            lcd_temp = tr
                            if (float(tr) > 0): #check if temp is reasonable, ie not a misread
                                Temp_Has_Changed = False
                                if abs(float(tr) - T_Compensation) >=1 : 
                                    T_Compensation=float(tr) 
                                    Temp_Has_Changed = True
                            #print("Temp Compensation Change is " + "True " if Temp_Has_Changed else "False ") 
                        pincontrol.flash('goodread')
                    except:
                        sensor['Reading'] = ['bad']
                        sensor['Error'] += 1 #increment error counter for sensor. upon exceeding threshold, it is reported in error log
                elif sensor['SensorType'] in ('20','30','40'):
                    
                    try:
                        device.set_i2c_address(int(sensor['Addr']))
                        time.sleep(0.5)
                        try:
                            stat = string.split(device.query("I"), ",")[1]
                        except:
                            stat='??'
                        #print("Status of addr " + sensor["Addr"] + " : " + stat) 

                        if (stat=='??'):
                            dline='Sensor at addr ' + sensor['Addr'] + ' Not Connected'
                            sensor['Reading']=['bad']
                            sensor['Error'] +=1
                            lcd_DO = sensor['Reading'][0]
                            pincontrol.flash('badread')
                            
                        else:
                            if Temp_Has_Changed:
                                T_Compensation=float(lcd_temp) 
                                try:
                                    str_cmd='T,' + str(T_Compensation)
                                    dline = device.query('T,' + str(T_Compensation))
                                    time.sleep(0.3)
                                    str_resp=device.query('T,?')
                                    print("Sensor Temp Comp has been set:" + dline + " ; " + str_cmd + " ; " + str_resp )
                                except:
                                    pass
                            if sensor['SensorType'] == '20': #salinity compensation applies only to DO probe
                                if EC_Has_Changed:
                                    try:
                                        dline = device.query('S,' + str(S_Compensation)+',ppt')
                                        time.sleep(0.3)
                                        print("DO Salinity Comp has been set")
                                    except:
                                        pass
                            dline = device.query('R')
                            ofst=(len(dline) - 11)*(-1)
                            agrdg = dline.strip()[ofst:].split(",")
                            dict_reading =  perse_reading(int(sensor['SensorType']),agrdg)
                            tr = dict_reading['default']
#                            tr=dline.strip()[ofst:]
                            print("Sensor Settints : " + device.query('o,?'))
                            print('Reading for sensor ' + sensor['SensorType']  + ' aggregated : ' + ",".join(agrdg) )
                            if sensor['SensorType'] == '20':
#                                if not (ValidReadings["OxygenMin"] <= float(tr) <= ValidReadings["OxygenMax"]): tr='0.0'
                                lcd_DO = tr
                                sensor['Reading'] = dict_reading
                            if sensor['SensorType'] =='30':
#                                if not (ValidReadings["pHMin"] <= float(tr) <= ValidReadings["pHMax"]): tr='0.0'
                                lcd_pH = tr
                                sensor['Reading'] = dict_reading
                            if sensor['SensorType'] == '40':
                                print('Salinity Reading  aggregated : ' + ",".join(agrdg))
#                                if not (ValidReadings["SalinityMin"] <= float(tr) <= ValidReadings["SalinityMax"]): tr='0.0'
                                lcd_EC = tr 
                                sensor['Reading'] = dict_reading
                                EC_Has_Changed = False
                                if abs(float(tr) - S_Compensation) >=1 : 
                                    EC_Has_Changed = True
                                    S_Compensation=float(tr) 

                            sensor['Error'] = 0
                            pincontrol.flash('goodread')
                    except:
                        sensor['Reading'] = perse_reading(int(sensor['SensorType']),['bad'])
                        sensor['Error'] += 1
                elif (sensor['SensorType'] == '50'): #water level using sonar
                    lcd_string("read analog",LCD_LINE_2)
                    WL_Reading=readWL(arduino,"sonar")
                    if not (WL_Reading.startswith('Err')):
                        sensor['Reading'] = [WL_Reading]
                        pincontrol.flash('goodread')
                    else:
                        sensor['Reading'] = ['bad']
                        sensor['Error'] += 1
                elif (sensor['SensorType'] == '55'): #water level using pressure sensor
                    lcd_string("read analog",LCD_LINE_2)
                    WP_Reading = readWL(arduino,"pressure")
                    if not (WP_Reading =='Error'):
                        sensor['Reading'] = [WP_Reading]
                        pincontrol.flash('goodread')
                    else:
                        sensor['Reading'] = ['bad']
                        sensor['Error'] += 1
                   
            
                else:   #sensor type not determined
                    sensor['Reading'] = ['bad']
                    sensor['Error'] += 1
            #end of for loop

    else:   #probes have jammed
        lcd_string("Error....", LCD_LINE_1)
        lcd_string("Gantry jam", LCD_LINE_2)
        print('gantry jam detected, cannot continue')
        pincontrol.set_alarm(1)
                
    lcd_string("Oxygen : " + lcd_DO, LCD_LINE_1)
    lcd_string("Temp :   " + lcd_temp, LCD_LINE_2)
    if DEBUGMODE:
            print("DO : " + lcd_DO + "; pH : "+ lcd_pH + "; EC : " + lcd_EC + "; temp degC: " + lcd_temp)
            print( "wtr lvl: " + WL_Reading + ", wtr lvl(P): " + WP_Reading )

    if (probes.getProbePosition() <> 'jam'):
        response = upload_data(pincontrol)
        if DEBUGMODE:
            print("Upload Data Response = " + str(response[0]))
        if response[0] == 0 and response[1] > 0: #no successful writes
            internetOnline=False
            if DEBUGMODE:
                print("webserver not accessible, going to offline mode")
        if response[2]   > 0: #all data was written offline without attempting to connect
            offlineCtr += 1
            if offlineCtr >= int(config["MAXOFFLINE"]):
                offlineCtr = 0
                internetOnline = True
        if response[1] + response[2] > 0 : #some data was written offline upone failed connection
            emptyOfflineData = False 
        log_errors(pincontrol)
    probe_result = probes.raise_probes()
    if DEBUGMODE: print('Now in waiting mode')
    lcd_string("Oxygen : " + lcd_DO, LCD_LINE_1,LCD_NOBL)
    lcd_string("Temp :   " + lcd_temp, LCD_LINE_2,LCD_NOBL)

def int_callback(channel):
        global intmode_button

        if GPIO.input(channel): #interactive req button has been pressed
                lcd_string("Oxygen : " + lcd_DO, LCD_LINE_1)
                lcd_string("Temp :   " + lcd_temp, LCD_LINE_2)
                print("IMode pressed")
                intmode_button =time.time()
        else: #button has been released. now check if it has been pressed for 3 secs
                print("IMode released")
                lcd_string("Oxygen : " + lcd_DO, LCD_LINE_1,LCD_NOBL)
                lcd_string("Temp :   " + lcd_temp, LCD_LINE_2,LCD_NOBL)
                time_rlsd = time.time()
                time_diff = time_rlsd - intmode_button
                if (time_diff > 3 and time_diff<10):
                        interactive_mode()
def interactive_mode():
#interactive mode allows manual control of boom using buttons
#    button on pin 23 to raise boom
#    button on pin 24 to lower boom
#if idle for 10 seconds raise boom and exit i mode 
        global probes, pincontrol, interactive_flag

        if enable_interactive:
            interactive_flag=True
            print("interactive mode requested")
            lcd_string("boom ctrl", LCD_LINE_1)
            lcd_string("up; down", LCD_LINE_2)
            pincontrol.flash("EnterIMode")
            time.sleep(0.5)
            intTime=time.time() #if user is idle for more than 10 seconds exit interactive mode
            while True:
                if GPIO.input(23):
                    pbstate=1
                    while pbstate:
                        print("probes up requested")
                        lcd_string("boom up",LCD_LINE_2)
                        intTime=time.time() 
                        probes.raise_probes(1)
                        print("raising probes 1 sec")
                        pbstate = GPIO.input(23)
                        if pbstate==False :
                            print("Probes up button released")
                            break
#                    if not(probes.getProbePosition=="down"): probes.halt_probes()
                elif GPIO.input(24):
                    pbstate=1
                    while pbstate:
                        print("probes down requested")
                        lcd_string("boom down",LCD_LINE_2)
                        intTime=time.time()
                        probes.lower_probes(1)
                        print("Lowering probes 1 sec")
                        pbstate = GPIO.input(24)
                        if pbstate==False :
                            print("Probes down button released")
                            break

#                    if not(probes.getProbePosition=="up"): probes.halt_probes()
                time.sleep(0.1)
                if (time.time()-intTime > 20):
                    print("timeout....exiting imode")
                    lcd_string("Timeout",LCD_LINE_1)
                    lcd_string("Exiting",LCD_LINE_2)
                    pincontrol.flash("ExitIMode")
                    interactive_flag=False
                    time.sleep(0.5)
                    probes.raise_probes()
                    lcd_string("Oxygen : " + lcd_DO, LCD_LINE_1,LCD_NOBL)
                    lcd_string("Temp :   " + lcd_temp, LCD_LINE_2,LCD_NOBL)
                    break
                   
                
        else:
            print("interactive mode disabled...system busy")
            pincontrol.flash("IModeError")
            time.sleep(0.5)
def set_sensor_options(device,sensor_type):
    if sensor_type == 20:
        DOoptions = config['DOSENSOROPTIONS'].split(',')
        qres = device.query("O,%,1" if 'PS' in DOoptions else "o,%,0") #set EC sensor conductivity option to  off
        qres = device.query("O,MG,1" if 'MG' in DOoptions else "o,MG,0") #set EC sensor conductivity option to  off

    elif sensor_type == 40:
        ECoptions = config['ECSENSOROPTIONS'].split(',')
        qres = device.query("O,EC,1" if 'EC' in ECoptions else "o,EC,0") #set EC sensor conductivity option to  off
        qres = device.query("O,SG,1" if 'SG' in ECoptions else "o,SG,0") #set EC sensor conductivity option to  off
        qres = device.query("O,S,1" if 'S' in ECoptions else "o,S,0") #set EC sensor conductivity option to  off
        qres = device.query("O,TDS,1" if 'TDS' in ECoptions else "o,TDS,0") #set EC sensor conductivity option to  off
    else:
        pass
#        print("Sensor Options : for type " + str(sensor_type) + " : " + device.query("o,?"))
def main():
    global DEBUGMODE, sensors, probes, pincontrol, enable_interactive, bus, LCD_ATTACHED, T_Compensation, S_Compensation
    debg="NO"
    try:
        script, debg = argv
    except:
        debg=""
    bus = smbus.SMBus(1) # Rev 2 Pi uses 1
    try:
        lcd_init()
        LCD_ATTACHED=True
    except:
        LCD_ATTACHED=False

    DEBUGMODE = (debg.upper() == "DEBUG")
    if LCD_ATTACHED:
        lcd_string('initializing...',LCD_LINE_1)
        lcd_string('',LCD_LINE_2)
    readConfig()
    offlineflag=setOfflineDataFlag()
    str1="DEBUG " if DEBUGMODE else "Production "
    log_status("System Startup on " + str1 + "mode" )
    if not DEBUGMODE:
        time.sleep(10.0) #give tome for I2C bus to be initialized
    pincontrol=devio()
    for i in range(1,3):
        pincontrol.set_op(1)
        time.sleep(0.2)
        pincontrol.set_op(0)
        time.sleep(0.1)            
    pincontrol.resetall()
    device = AtlasI2C() 	# creates the I2C port object, specify the address or bus if necessary
    probes=motor(config['MOTOR'].lower(),int(config['MOTORRUNTIME']))
    probes.sysEnabled(config['MOTOR'].lower())
    if (config['MOTOR'].lower() == 'yes'):
        if LCD_ATTACHED:
            lcd_string('boom motorized..',LCD_LINE_2)
        
#        if (config['MOTOR'].upper() == 'YES'): probes.sysEnabled('yes')
    init1Wire()
    try:
        arduino = serial.Serial(config['ARDUINO'],9600,timeout=3)
        time.sleep(2) #allow the link with arduino to be established
        print("Arduino Found")
    except:
        arduino=""
        print("No Arduino at " + config['ARDUINO'])

    probes.setarduino(arduino)

    dline = "No reading yet"
    device.set_i2c_address(int(config['DEVICEADDR']))
    GPIO.setup(18,GPIO.IN)
    GPIO.setup(23,GPIO.IN)
    GPIO.setup(24,GPIO.IN)
    GPIO.add_event_detect(18,GPIO.BOTH,callback=int_callback,bouncetime=100)
    if LCD_ATTACHED:
        lcd_string(commands.getoutput('hostname -I'),LCD_LINE_2)
        time.sleep(3)
    if DEBUGMODE:
        print(">> Any commands entered are passed to the board via I2C except:")
        print(">> Pin feature,on/off to control GPIO for features link,operation,alarm,oxygen eg Pin,set,alarm,on; Pin,check,oxygen")
        print(">>   List_addr lists the available I2C addresses.")
        print(">>   Address,xx changes the I2C address the Raspberry Pi communicates with.")
        print(">>   Poll,xx.x command continuously polls the board every xx.x seconds")
        print(" where xx.x is longer than the %0.2f second timeout." % AtlasI2C.long_timeout)
        print(">> Pressing ctrl-c will stop the polling")

        T_Compensation=float(config['TCOMP'])
        S_Compensation=float(config['SCOMP'])

        #        if DEBUGMODE:
        while True:
            input = raw_input("Enter command: ")
            if input.upper().startswith("LIST_ADDR"):
              devices = device.list_i2c_devices()
              for i in range(len (devices)):
                print(devices[i])

              # address command lets you change which address the Raspberry Pi will poll
            elif input.upper().startswith("ADDRESS"):
              addr = int(string.split(input, ',')[1])
              device.set_i2c_address(addr)
              print("I2C address set to " + str(addr))

            # continuous polling command automatically polls the board
            elif input.upper().startswith("POLL"):
              delaytime = float(string.split(input, ',')[1])

              # check for polling time being too short, change it to the minimum timeout if too short
              if (delaytime < AtlasI2C.long_timeout):
                print("Polling time is shorter than timeout, setting polling time to %0.2f" % AtlasI2C.long_timeout)
                delaytime = AtlasI2C.long_timeout
              # get the information of the board you're polling
              try:
                info = string.split(device.query("I"), ",")[1]
              except:
                  info = "??"
              print("Polling %s sensor every %0.2f seconds, press ctrl-c to stop polling" % (info, delaytime))
              log_status("Begin polling sensors in DEBUG")
              probes.resetProbePosition()
              try:
                while True:
                  pollDevice(device, lcd1, delaytime, pincontrol, probes, arduino)
                  time.sleep(int(delaytime))
              except KeyboardInterrupt: 		# catches the ctrl-c command, which breaks the loop above
                log_status("Polling interrupted through Ctr-C")
                print("Continuous polling stopped")
                if LCD_ATTACHED:
                    lcd_string("stopped...",LCD_LINE_1)
                    lcd_string("Clearing...",LCD_LINE_2)
                    time.sleep(1.0)
                    lcd_string("",LCD_LINE_1,LCD_NOBL)
                    lcd_string("",LCD_LINE_2,LCD_NOBL)
            elif input.upper().startswith("TRANSMIT"): #Transmit data to server. expects arguments as tds,ec,s,mg,%,T,WL for sensor types 40,20,10,50
                if len(string.split(input, ',')) == 1:  #no data specified, defaults will be used
                    tstdata = ['TRANSMIT','150','35000','35','9.1','145','25.2','155']
                else :
                    tstdata = string.split(input, ',')

                for sensor in sensors:
                    if sensor['SensorType'] == '40': # salinity
                        sensor['Reading'] = perse_reading(int(sensor['SensorType']), [tstdata[1],tstdata[2],tstdata[3]])
                    elif sensor['SensorType'] == '20': # DO
                        sensor['Reading'] = perse_reading(int(sensor['SensorType']), [tstdata[4],tstdata[5]])
                    elif sensor['SensorType'] == '10': # Temp
                        sensor['Reading'] = perse_reading(int(sensor['SensorType']), [tstdata[6]])
                    elif sensor['SensorType'] == '50': # WL
                        sensor['Reading'] = perse_reading(int(sensor['SensorType']), [tstdata[7]])
                print tstdata
                print(upload_data(pincontrol))

            elif input.upper().startswith("OFFLINE"): #Transmit offline data to server
                print(upload_offline_data())

            elif input.upper().startswith("CR"): #continuous reading from atlas probe on defined address
              try:
                while True:
                  print(device.query('r'))
                  time.sleep(3.0)
              except KeyboardInterrupt:
                print("polling stopped....")
              except IOError:
                print("Query failed \n - Address may be invalid, use List_addr command to see available addresses")
              
            elif input.upper().startswith("TEMP"): #read temoperature
                for sensor in sensors:
                    if ((sensor['SensorType']  == '10') and (len(sensor['File']) > 2)):
                        print("Temperature in C : " + read_temp(sensor['File']))
                        break

            elif input.upper().startswith("CLB"): #prepare sensor for calibration

                clb_type = '40' if len(string.split(input, ',')) == 1  else string.split(input, ',')[1].upper()
                if clb_type == "40":
                    qres = device.query("O,EC,1") #set EC sensor conductivity option to  on
                    qres = device.query("O,SG,1") #set EC sensor specific gravity option to  on
                    qres = device.query("O,TDS,1") #set EC Total Dissolved Substances option to  on
                    qres = device.query("O,S,1") #set EC sensor salinity option to  on
                    print ("Ok, sensor is ready for calibration process. start with CAL,DRY ")
                    print("Sensor Options : " + device.query('o,?'))
                if clb_type == "20":
                    qres = device.query("O,MG,1") #set DO sensor MG option to  on
                    qres = device.query("O,%,1") #set EC sensor specific gravity option to  on
                    print ("Ok, sensor is ready for calibration process. start with CAL ")
                    print("Sensor Options : " + device.query('o,?'))

            elif input.upper().startswith("SETOPT"): #set sensor options from config['sensor options entry']
                s_type = 40 if len(string.split(input, ',')) == 1  else int(string.split(input, ',')[1].upper())
                set_sensor_options(device,s_type)

            elif input.upper().startswith("PB"): #test push buttons
                but1 = GPIO.input(18)
                but2 = GPIO.input(23)
                but3 = GPIO.input(24)
                str1 = ": on," if but1==1 else ": off,"
                str1 += ": on," if but2==1 else ": off,"
                str1 += ": on," if but3==1 else ": off,"
                str1 = "Buttuns state " + str1
                print("Entering push-button testing modes.")
                print("This allows testing of buttons on GPIO18, 23, and 24.")
                print("Press Ctrl+C to exit this mode")
                
                print(str1)
                
                try:
                    while True:
                        but_read = GPIO.input(18)                                    
                        if (not(but1 == but_read)):
                            str1 = ": on," if but_read else ": off,"
                            str1 = "Button 1 state changed to "  +str1
                            print(str1)
                            but1=but_read
                        but_read = GPIO.input(23)                                    
                        if (not(but2 == but_read)):
                            str1 = ": on," if but_read else ": off,"
                            str1 = "Button 2 state changed to "  +str1
                            print(str1)
                            but2 = but_read
                        but_read = GPIO.input(24)                                    
                        if (not(but3 == but_read)):
                            str1 = ": on," if but_read==1 else ": off,"
                            str1 = "Button 3 state changed to "  +str1
                            print(str1)
                            but3 = but_read
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    print("Button Testing ended")
                            



            elif input.upper().startswith("PIN"):
              pincmd = string.split(input,',')
              if pincmd[1].upper() == 'SET':
                pinstate=1 if pincmd[3] == 'on' else 0
                if pincmd[2].upper() == 'LINK':
                    pincontrol.set_link(pinstate)
                elif pincmd[2].upper() == 'OPERATION':
                    pincontrol.set_op(pinstate)
                elif pincmd[2].upper() == 'ALARM':
                    pincontrol.set_alarm(pinstate)
                elif pincmd[2].upper() == 'OXYGEN':
                    pincontrol.flip_oxygen(pinstate)
              else:
                print('Data Link state is ' + pincontrol.get_link())
                print('Operation state is ' + pincontrol.get_op())
                print('Alarm state is ' + pincontrol.get_alarm())
                print('Oxygen Supply state is ' + pincontrol.get_oxygenstate())

            elif input.upper().startswith("MOTOR"):
              pincmd = string.split(input,',')
              if pincmd[1].upper() == 'RUN':
                runsecs=int(pincmd[3])
                if pincmd[2].upper() == 'UP':
                    probes.raise_probes(runsecs)
                elif pincmd[2].upper() == 'DOWN':
                    probes.lower_probes(runsecs)
                else:
                    print('probe position is : ' + probes.getProbePosition())
              else:
                  print('probe position is : ' + probes.getProbePosition())

            elif input.upper().startswith('MX'): #short command MX for motor Control
                pincmd = string.split(input,',')
                runsecs = 0 if len(pincmd) == 1  else int(pincmd[1])
                if pincmd[0].upper()=='MXUP':
                    print(probes.raise_probes(runsecs))
                elif pincmd[0].upper()=='MXDN':
                    print(probes.lower_probes(runsecs))
                else:
                    print('Cannot process request ' + input.upper() + ' , probe position is : ' + probes.getProbePosition())

            # if not a special keyword, pass commands straight to board
            elif input.upper().startswith('WL'): #get ultrasonic or pressure readings
                pincmd = string.split(input,',')
                stype = 'sonar' if len(pincmd) == 1  else pincmd[1].lower()
                print("Water Level sensor reading : " + readWL(arduino, stype))

            else:
              if len(input) == 0:
                print("Please input valid command.")
              else:
                try:
                  print(device.query(input))
                except IOError:
                  print("Query failed \n - Address may be invalid, use List_addr command to see available addresses")
    else:
        for sensor in sensors:
            if sensor['SensorType'] == '20': #DO sensor
                device.set_i2c_address(int(sensor['Addr']))
                time.sleep(0.5)
                set_sensor_options(20)
                log_status("Current Temp comp. on DO sensor " + sensor['SensorID'] + " : " + device.query("T,?") + "; Salinity Compensation : " + device.query("S,?"))
                qres = device.query("T," + config['TCOMP'])
                qres = device.query("S," + config['SCOMP'] + ",ppt")
                log_status("Set Temp comp. on DO sensor " + sensor['SensorID'] + " : " + device.query("T,?") + "; Salinity Compensation : " + device.query("S,?") + "; Output Options : " + device.query("O,?"))
            if sensor['SensorType'] == '30': #pH sensor
                device.set_i2c_address(int(sensor['Addr']))
                time.sleep(0.5)
                log_status("Current Temp comp. on pH sensor " + sensor['SensorID'] + " : " + device.query("T,?") )
                qres = device.query("T," + config['TCOMP'])
                log_status("Set Temp comp. on pH sensor " + sensor['SensorID']  + " : " + device.query("T,?"))
              
            if sensor['SensorType'] == '40': #EC 
                device.set_i2c_address(int(sensor['Addr']))
                time.sleep(0.5)
                set_sensor_options(40)
                log_status("Current Temp comp. on EC sensor " + sensor['SensorID'] + " : " + device.query("T,?") )
                qres = device.query("T," + config['TCOMP'])
                log_status("Set Temp comp. on EC sensor " + sensor['SensorID']  + " : " + device.query("T,?") + "; Output Options : " + device.query("O,?"))
          
        log_status("Begin polling sensors in production mode")
        try:
          device.query("Address,97")
          time.sleep(0.5)
          DO_Opt = device.query("O,?")
          device.query("Address,100")
          time.sleep(0.5)
          EC_Opt = device.query("O,?")
          print("Output Options : DO = " + DO_Opt + "; EC = " + EC_Opt)
        except:
          pass

        timeInterval = int(config["POLLINTERVAL"])
        try:
            while True:
                if (not interactive_flag):
                    enable_interactive=False
                    pollDevice(device, None, int(config["POLLINTERVAL"]),pincontrol, probes, arduino)
                    enable_interactive=True
                    print("you may enter interactive now...")
                    time.sleep(timeInterval)
                else:
                    #                            print("Cannot start polling ")
                    time.sleep(1)

        except KeyboardInterrupt: 		# catches the ctrl-c command, which breaks the loop above
            log_status("Polling interrupted, system shutting down")
            lcd_string("Shut down...", LCD_LINE_1)
            lcd_string("", LCD_LINE_2)
            time.sleep(2)
            lcd_string("Stopped...", LCD_LINE_1,LCD_NOBL)
            print("interrupted")                    		

if __name__ == '__main__':
    main()

