#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Based on: https://github.com/thedancomplex/pydynamixel
# */

import sys
sys.path.append('/home/archr/hubo_simulation/files/dynamixel')
import os
import dynamixel
import serial_stream
import time
import random
import sys
import subprocess
import optparse
import yaml
import dynamixel_network
import numpy as np


# Hubo-ach stuff
import hubo_ach as ha
import ach
from ctypes import *

window = 10
#left arm
s0 = 0;
s1 = 0;
e0 = 0;
e1 = 0;
w0 = 0;
w1 = 0;
w2 = 0;
s0List = [0, 0, 0, 0, 0];
s1List = [0, 0, 0, 0, 0];
e0List = [0, 0, 0, 0, 0];
e1List = [0, 0, 0, 0, 0];
w0List = [0, 0, 0, 0, 0];
w1List = [0, 0, 0, 0, 0];
w2List = [0, 0, 0, 0, 0];
lf2List = [0, 0, 0, 0, 0];
#right arm
s0_r = 0;
s1_r = 0;
e0_r = 0;
e1_r = 0;
w0_r = 0;
w1_r = 0;
w2_r = 0;
s0List_r = [0, 0, 0, 0, 0];
s1List_r = [0, 0, 0, 0, 0];
e0List_r = [0, 0, 0, 0, 0];
e1List_r = [0, 0, 0, 0, 0];
w0List_r = [0, 0, 0, 0, 0];
w1List_r = [0, 0, 0, 0, 0];
w2List_r = [0, 0, 0, 0, 0];
rf2List = [0, 0, 0, 0, 0];
#Assigning from profile
Ax = [1024,2047]

# feed-forward will now be refered to as "state"
#state = ha.HUBO_STATE()  #not used
# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()
# Get the current feed-forward (state) 
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
[statuss, framesizes] = r.get(ref, wait=False, last=True)


def rad2dyn(rad):
    return np.int(np.floor( (rad + np.pi)/(2.0 * np.pi) * 4096 ))

def dyn2rad(en):
    return ((en*2.0*np.pi)/Ax[0]) - np.pi

def main(settings): 
    portName = settings['port']
    baudRate = settings['baudRate']
    highestServoId = settings['highestServoId']

    # Establish a serial connection to the dynamixel network.
    # This usually requires a USB2Dynamixel
    serial = serial_stream.SerialStream(port=portName, baudrate=baudRate, timeout=1)
    net = dynamixel_network.DynamixelNetwork(serial)
    
    # Ping the range of servos that are attached
    print "Scanning for Dynamixels..."
    net.scan(1, highestServoId)
    
    myActuators = []
    for dyn in net.get_dynamixels():
        print dyn.id
        myActuators.append(net[dyn.id])
    
    if not myActuators:
      print 'No Dynamixels Found!'
      sys.exit(0)
    else:
      print "...Done"
    
    for actuator in myActuators:
        actuator.moving_speed = 80
        actuator.synchronized = True
        actuator.torque_enable = False
        actuator.torque_limit = 0
        actuator.max_torque = 0

    #create initial data for averaging filter lists
    while ( (len(s0List)<window) and (len(s1List)<window) ):
        actuator.read_all()
        for actuator in myActuators:
                '''if ( actuator.id == 1):   
                    s0List.append(dyn2rad(actuator.current_position));             
                    s0 = np.mean(s0List);'''
                if ( actuator.id == 2):     
                    s1List.append(dyn2rad(actuator.current_position)+3.14);             
                    s1 = np.mean(s1List);                       
                if ( actuator.id == 3): 
                    e0List.append(dyn2rad(actuator.current_position));             
                    e0 = np.mean(e0List);                                                 
                if ( actuator.id == 4):   
                    e1List.append(dyn2rad(actuator.current_position));             
                    e1 = np.mean(e1List);                        
                if ( actuator.id == 5):   
                    w0List.append(dyn2rad(actuator.current_position));             
                    w0 = np.mean(w0List);                         
                if ( actuator.id == 6):      
                    w1List.append(dyn2rad(actuator.current_position));             
                    w1 = np.mean(w1List);                    
                '''if ( actuator.id == 7):                
                    w2List.append(dyn2rad(actuator.current_position));             
                    w2 = np.mean(w2List);
                if ( actuator.id == 18):  
                    lf2List.append(dyn2rad(actuator.current_position));             
                    lf2 = np.mean(lf2List);'''                          
                if ( actuator.id == 12):     
                    s1List_r.append(dyn2rad(actuator.current_position));             
                    s1_r = np.mean(s1List_r);                       
                if ( actuator.id == 13): 
                    e0List_r.append(dyn2rad(actuator.current_position));             
                    e0_r = np.mean(e0List_r);                                                 
                if ( actuator.id == 14):   
                    e1List_r.append(dyn2rad(actuator.current_position));             
                    e1_r = np.mean(e1List_r);                        
                if ( actuator.id == 15):   
                    w0List_r.append(dyn2rad(actuator.current_position));             
                    w0_r = np.mean(w0List_r);                         
                if ( actuator.id == 16):      
                    w1List.append(dyn2rad(actuator.current_position));             
                    w1_r = np.mean(w1List_r);                    
                '''if ( actuator.id == 17):                
                    w2List_r.append(dyn2rad(actuator.current_position));             
                    w2_r = np.mean(w2List_r);                        
                if ( actuator.id == 18):  
                    rf2List.append(dyn2rad(actuator.current_position));             
                    rf2 = np.mean(rf2List);  '''                            
        time.sleep(.01) #200 Hz refresh rate


    #read data in real-time           
    timeout = time.time() + 60*180   #Terminate 180 minutes from now
    while True:    
        actuator.read_all()
    	for actuator in myActuators:
                #left arm
                '''if ( actuator.id == 1):   
                    s0List.append(dyn2rad(actuator.current_position));             
                    s0List.pop(0);
                    s0 = np.mean(s0List);
                    ref.ref[ha.LSY] = s0;'''
                if ( actuator.id == 2):     
                    s1List.append(dyn2rad(actuator.current_position));             
                    s1List.pop(0);
                    s1 = np.mean(s1List);                       
                    ref.ref[ha.LSP] = s1;
                if ( actuator.id == 3): 
                    e0List.append(dyn2rad(actuator.current_position));             
                    e0List.pop(0);
                    e0 = np.mean(e0List);                       
                    ref.ref[ha.LSR] = e0;                           
                if ( actuator.id == 4):   
                    e1List.append(dyn2rad(actuator.current_position));             
                    e1List.pop(0);
                    e1 = np.mean(e1List);                        
                    ref.ref[ha.LEB] = e1 - 1.73; #offset of 1.73
                if ( actuator.id == 5):   
                    w0List.append(dyn2rad(actuator.current_position));             
                    w0List.pop(0);
                    w0 = np.mean(w0List);                         
                    ref.ref[ha.LHY] = w0;
                if ( actuator.id == 6):      
                    w1List.append(dyn2rad(actuator.current_position));             
                    w1List.pop(0);
                    w1 = np.mean(w1List);                    
                    ref.ref[ha.LHP] = w1;
                '''if ( actuator.id == 7):                
                    w2List.append(dyn2rad(actuator.current_position));             
                    w2List.pop(0);
                    w2 = np.mean(w2List);  
                    ref.ref[ha.LWR] = w2;                
                    #for debugging: print "[ID] radian value = %f" %dyn2rad(actuator.current_position) 
                    print "LSY, LSP, LSR, LWR values = %f %f %f %f" %(ref.ref[ha.LSY], ref.ref[ha.LSP], ref.ref[ha.LSR], ref.ref[ha.LWR])
                if ( actuator.id == 8):              
                    lf2List.append(dyn2rad(actuator.current_position));             
                    lf2List.pop(0);
                    lf2 = np.mean(lf2List);                              
                    ref.ref[ha.LF2] = lf2 - 470;
                    print "LF2 value = %f" %(ref.ref[ha.LF2])'''
                #right arm
                '''if ( actuator.id == 11):   
                    s0List_r.append(dyn2rad(actuator.current_position));             
                    s0List_r.pop(0);
                    s0_r = np.mean(s0List_r);
                    ref.ref[ha.RSY] = s0_r;'''                    
                if ( actuator.id == 12):     
                    s1List_r.append(dyn2rad(actuator.current_position));             
                    s1List_r.pop(0);
                    s1_r = np.mean(s1List_r);                       
                    ref.ref[ha.RSP] = s1_r;
                if ( actuator.id == 13): 
                    e0List_r.append(dyn2rad(actuator.current_position));             
                    e0List_r.pop(0);
                    e0_r = np.mean(e0List_r);                       
                    ref.ref[ha.RSR] = e0_r;                           
                if ( actuator.id == 14):   
                    e1List_r.append(dyn2rad(actuator.current_position));             
                    e1List_r.pop(0);
                    e1_r = np.mean(e1List_r);                        
                    ref.ref[ha.REB] = e1_r - 1.73; #offset of 1.73
                if ( actuator.id == 15):   
                    w0List_r.append(dyn2rad(actuator.current_position));             
                    w0List_r.pop(0);
                    w0_r = np.mean(w0List_r);                         
                    ref.ref[ha.RHY] = w0_r;
                if ( actuator.id == 16):      
                    w1List_r.append(dyn2rad(actuator.current_position));             
                    w1List_r.pop(0);
                    w1_r = np.mean(w1List_r);                    
                    ref.ref[ha.RHP] = w1_r;      
                '''if ( actuator.id == 17):                
                    w2List_r.append(dyn2rad(actuator.current_position));             
                    w2List_r.pop(0);
                    w2_r = np.mean(w2List_r);  
                    ref.ref[ha.RWR] = w2_r;                
                    #for debugging: print "[ID] radian value = %f" %dyn2rad(actuator.current_position) 
                    print "RSY, RSP, RSR, RWR values = %f %f %f %f" %(ref.ref[ha.RSY], ref.ref[ha.RSP], ref.ref[ha.RSR], ref.ref[ha.RWR])
                if ( actuator.id == 18):              
                    rf2List.append(dyn2rad(actuator.current_position));             
                    rf2List.pop(0);
                    rf2 = np.mean(rf2List);                              
                    ref.ref[ha.RF2] = rf2 - 470;
                    print "RF2 value = %f" %(ref.ref[ha.RF2])'''                                  
                r.put(ref)
        time.sleep(.02) #200 Hz refresh rate
        if time.time() > timeout:
            print "Time out, hubo-ach channels closing."
            break

def validateInput(userInput, rangeMin, rangeMax):
    '''
    Returns valid user input or None
    '''
    try:
        inTest = int(userInput)
        if inTest < rangeMin or inTest > rangeMax:
            print "ERROR: Value out of range [" + str(rangeMin) + '-' + str(rangeMax) + "]"
            return None
    except ValueError:
        print("ERROR: Please enter an integer")
        return None
    
    return inTest

if __name__ == '__main__':
    
    parser = optparse.OptionParser()
    parser.add_option("-c", "--clean",
                      action="store_true", dest="clean", default=False,
                      help="Ignore the settings.yaml file if it exists and \
                      prompt for new settings.")
    
    (options, args) = parser.parse_args()
    
    # Look for a settings.yaml file
    settingsFile = 'settings.yaml'
    if not options.clean and os.path.exists(settingsFile):
        with open(settingsFile, 'r') as fh:
            settings = yaml.load(fh)
    # If we were asked to bypass, or don't have settings
    else:
        settings = {}
        if os.name == "posix":
            portPrompt = "Which port corresponds to your USB2Dynamixel? \n"
            # Get a list of ports that mention USB
            try:
                possiblePorts = subprocess.check_output('ls /dev/ | grep -i usb',
                                                        shell=True).split()
                possiblePorts = ['/dev/' + port for port in possiblePorts]
            except subprocess.CalledProcessError:
                sys.exit("USB2Dynamixel not found. Please connect one.")
                
            counter = 1
            portCount = len(possiblePorts)
            for port in possiblePorts:
                portPrompt += "\t" + str(counter) + " - " + port + "\n"
                counter += 1
            portPrompt += "Enter Choice: "
            portChoice = None
            while not portChoice:                
                portTest = raw_input(portPrompt)
                portTest = validateInput(portTest, 1, portCount)
                if portTest:
                    portChoice = possiblePorts[portTest - 1]

        else:
            portPrompt = "Please enter the port name to which the USB2Dynamixel is connected: "
            portChoice = raw_input(portPrompt)
    
        settings['port'] = portChoice
        
        # Baud rate
        baudRate = None
        while not baudRate:
            brTest = raw_input("Enter baud rate [Default: 1000000 bps]:")
            if not brTest:
                baudRate = 1000000
            else:
                baudRate = validateInput(brTest, 9600, 1000000)
                    
        settings['baudRate'] = baudRate
        
        # Servo ID
        highestServoId = None
        while not highestServoId:
            hsiTest = raw_input("Please enter the highest ID of the connected servos: ")
            highestServoId = validateInput(hsiTest, 1, 255)
        
        settings['highestServoId'] = highestServoId
        
        # Save the output settings to a yaml file
        with open(settingsFile, 'w') as fh:
            yaml.dump(settings, fh)
            print("Your settings have been saved to 'settings.yaml'. \nTo " +
                   "change them in the future either edit that file or run " +
                   "this example with -c.")
    
    main(settings)
