#!/usr/bin/env python

import rospy
import baxter_interface
# Hubo-ach stuff
import hubo_ach as ha
import ach
from ctypes import *
import time

s0 = 0;
s1= -0;
e0 = 0;
e1 = 0;
w0 = 0;
w1 = 0;
w2 = 0;
s0_r = 0;
s1_r = 0;
e0_r = 0;
e1_r = 0;
w0_r = 0;
w1_r = 0;
w2_r = 0;
# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()
# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()
# Get the current feed-forward (state) 
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
[statuss, framesizes] = r.get(ref, wait=False, last=True)

#enabling baxter
'''
print("Getting robot state... ")
rs = baxter_interface.RobotEnable(CHECK_VERSION)
print("Enabling robot... ")
rs.enable()
'''

rospy.init_node('Hello_Baxter')
limb = baxter_interface.Limb('left')
#limb_r = baxter_interface.Limb('right')
angles = limb.joint_angles()
#angles_r = limb_r.joint_angles()

#Initialize Left Arm
print angles
angles['left_s0']=0.0
angles['left_s1']=0.0
angles['left_e0']=0.0
angles['left_e1']=0.0
angles['left_w0']=0.0
angles['left_w1']=0.0
angles['left_w2']=0.0
limb.set_joint_positions(angles)

#Initialize Right Arm
'''print angles_r
angles_r['right_s0']=0.0
angles_r['right_s1']=0.0
angles_r['right_e0']=0.0
angles_r['right_e1']=0.0
angles_r['right_w0']=0.0
angles_r['right_w1']=0.0
angles_r['right_w2']=0.0
limb_r.set_joint_positions(angles_r)
time.sleep(3)'''


timeout = time.time() + 30 * 1 #Time to run script for
while True:
	time.sleep(0.01)
	r.get(ref)
	#left arm
	s0 = ref.ref[ha.LSY];
	s1 = ref.ref[ha.LSP];
	e0 = ref.ref[ha.LSR];
	e1 = ref.ref[ha.LEB];
	w0 = ref.ref[ha.LHY];
	w1 = ref.ref[ha.LHP];
	w2 = ref.ref[ha.LWR];
	angles['left_s0']=s0
	angles['left_s1']=s1
	angles['left_e0']=e0
	angles['left_e1']=e1
	angles['left_w0']=w0
	angles['left_w1']=w1
	angles['left_w2']=w2	
	# Right arm 
	'''s0_r = ref.ref[ha.RSY];
	s1_r = -ref.ref[ha.RSP];
	e0_r = ref.ref[ha.RSR];
	e1_r = -ref.ref[ha.REB];
	w0_r = ref.ref[ha.RHY];
	w1_r = -ref.ref[ha.RHP];
	w2_r = ref.ref[ha.RWR];	
	angles['right_s0']=s0_r
	angles['right_s1']=s1_r
	angles['right_e0']=e0_r
	angles['right_e1']=e1_r
	angles['right_w0']=w0_r
	angles['right_w1']=w1_r
	angles['right_w2']=w2_r'''
	#set both limbs
	limb.set_joint_positions(angles)
	#limb_r.set_joint_positions(angles_r)	
	if time.time() > timeout:
		print "Time out. Quiting."
		break