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
# */


import termios, fcntl, sys, os #To allow for keyboard commands
import hubo_ach as ha
import ach
import sys
import time
from ctypes import *

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
#s.flush()
#r.flush()

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)

fd = sys.stdin.fileno()

oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)

oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

RWP = 0;
RSP = 0;
RSR = 0;
RSY = 0;
REB = 0;
NK1 = 0;
NKY = 0
NK2 = 0
try:
    while 1:
        try:
            c = sys.stdin.read(1); 
	    if c == 'a':
    		    RSP = RSP-0.1;
      		    ref.ref[ha.RSP] = RSP;
      		    r.put(ref)
    	    if c == 'z': 
      		    RSP = RSP+0.1;
      		    ref.ref[ha.RSP] = RSP;
      		    r.put(ref)
	    if c == 's':
    		    RSR = RSR-0.1;
      		    ref.ref[ha.RSR] = RSR;
      		    r.put(ref)
    	    if c == 'x': 
      		    RSR = RSR+0.1;
      		    ref.ref[ha.RSR] = RSR;
      		    r.put(ref)
	    if c == 'd':
    		    RSY = RSY-0.1;
      		    ref.ref[ha.RSY] = RSY;
      		    r.put(ref)
    	    if c == 'c': 
      		    RSY = RSY+0.1;
      		    ref.ref[ha.RSY] = RSY;
      		    r.put(ref)
	    if c == 'f':
    		    REB = REB-0.1;
      		    ref.ref[ha.REB] = REB;
      		    r.put(ref)
    	    if c == 'v': 
      		    REB = REB+0.1;
      		    ref.ref[ha.REB] = REB;
      		    r.put(ref)
	    if c == 'g':
    		    RWP = RWP-0.1;
      		    ref.ref[ha.RWP] = RWP;
      		    r.put(ref)
    	    if c == 'b': 
      		    RWP = RWP+0.1;
      		    ref.ref[ha.RWP] = RWP;
      		    r.put(ref)
            if c == '1': 
              RF1 = 1;
              RF2 = 1;              
              ref.ref[ha.RF1] = RF1;
              ref.ref[ha.RF2] = RF2;              
              r.put(ref)       
            if c == '2': 
              RF1 = -1;
              RF2 = -1;              
              ref.ref[ha.RF1] = RF1;
              ref.ref[ha.RF2] = RF2;              
              r.put(ref)      
            if c == '3': 
              NK1 = NK1 + 0.1           
              ref.ref[ha.NK1] = NK1;            
              r.put(ref)  
            if c == '4': 
              NKY = NKY + 0.1           
              ref.ref[ha.NKY] = NKY;            
              r.put(ref)    
            if c == '5': 
              NK2 = NK2 + 0.1           
              ref.ref[ha.NK2] = NK2;            
              r.put(ref)      
            if c == '6': 
              NK1 = NK1 - 0.1           
              ref.ref[ha.NK1] = NK1;            
              r.put(ref)  
            if c == '7': 
              NKY = NKY - 0.1           
              ref.ref[ha.NKY] = NKY;            
              r.put(ref)    
            if c == '8': 
              NK2 = NK2 - 0.1           
              ref.ref[ha.NK2] = NK2;            
              r.put(ref)                                                                                                 
   	    if c == 'q':
    		    break
        except IOError: pass
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

# Close the connection to the channels
r.close()
s.close()

