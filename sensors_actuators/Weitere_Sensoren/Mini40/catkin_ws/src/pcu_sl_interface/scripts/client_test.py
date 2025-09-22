#!/usr/bin/env python

import sys
import time

import rospy

import threading

from pcu_common_utils.AcquireRessource import AcquireRessource
from pcu_sl_interface.srv import SetState
from pcu_sl_interface.msg import ControllerState, JointsVector

        
class Node:
    def __init__(self):
        self.sema = threading.Semaphore(0)
        
        self.ar = AcquireRessource("/SL_RT_CORE/acquire")

        self.srv_set_state = rospy.ServiceProxy("/SL_RT_CORE/set_state", SetState)
        
        print("waiting for service...")
        self.srv_set_state.wait_for_service()
        print("ok!")
        

        self.qd_pub = rospy.Publisher("/SL_RT_CORE/qd_set", JointsVector, queue_size=1)
        
    def aq(self):
        print("Aquiring Core")
        if self.ar.acquire(1.0):
            print("Aquired!")
        else:
            print("timeout!")


    def run(self):
        
        
        #self.done = false 
        self.setState(ControllerState.STANDBY)
        
        self.aq()
        
        self.setState(ControllerState.TRA_VELOCITY)
        
        rate = rospy.Rate(10)
        c = 0
        
        #time.sleep(2)
        
        
        
        while self.ar.is_acquired and not rospy.is_shutdown():
            c += 0.1
            jv = JointsVector()
            jv.joint_data = [c, c*3.3]
            
            self.qd_pub.publish(jv)
            rate.sleep()
            
            
        self.ar.release()
        
        print("Broken")
                        
    def setState(self, nr):
        cs = ControllerState()
        cs.state = nr
        if not self.srv_set_state(cs).success:
            print("Failed to set State.")
        else:
            print("Setting state ok.")
        
rospy.init_node('client_test', anonymous=True)
node = Node()

t = threading.Thread(target=node.run)
t.start()

#node.run()
rospy.spin()










"""


id = generate_unique_id()
# Sends id to B using an action or a service
bond = bondpy.Bond("example_bond_topic", id)
bond.start()
if not bond.wait_until_formed(rospy.Duration(1.0)):
    raise Exception('Bond could not be formed')
# ... do things with B ... 
bond.wait_until_broken()
print "B has broken the bond"
"""
