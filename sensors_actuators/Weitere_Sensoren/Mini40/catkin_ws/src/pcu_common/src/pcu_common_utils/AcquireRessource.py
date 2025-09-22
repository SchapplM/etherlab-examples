#!/usr/bin/env python
# Action Server zur Organisation der Client-Verbindung zum Simulink-Modell

# Lucas Jürgens (BA), lucas.juergens@zubox.de, 2017-02
# Moritz Schappler, schappler@irt.uni-hannover.de
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

import rospy
import actionlib

import threading
from Queue import Queue

from pcu_common.msg import AcquireRessourceAction, AcquireRessourceGoal

class AcquirableRessource():
    def __init__(self, action_topic):
        
        self.acquired = False
        
        self._as = actionlib.SimpleActionServer(action_topic, AcquireRessourceAction, auto_start=False)
        
        self._as.register_goal_callback(self._goal_cb)
        self._as.register_preempt_callback(self._preempt_cb)
        
        self._as.start()
        
        self._aborted_cb = None
        self._acquired_cb = None
        
        t = threading.Thread(target=self._check_for_abort)
        t.start()
        
    def register_acquired_cb(self, cb):
        self._acquired_cb = cb
        
    def register_aborted_cb(self, cb):
        self._aborted_cb = cb
        
    def _check_for_abort(self):
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            if self.acquired:
                if not self._as.is_active():
                    self.acquired = False
                    if self._aborted_cb:
                        self._aborted_cb()
            rate.sleep()
                        
    def break_acquirement(self):
        if self.acquired:
            self._as.set_aborted()
            acquired = False
            
    def _goal_cb(self):
        self._as.accept_new_goal()
        self.acquired = True
        if self._acquired_cb:
            self._acquired_cb()

        
    def _preempt_cb(self):
        self._as.set_aborted()
            
class AcquireRessource():
    def __init__(self, action_topic):
        self._ac = actionlib.SimpleActionClient(action_topic, AcquireRessourceAction)
        self._queue = Queue()   # Using a queue here, because python 2 doesnt know a timeout for semaphore.wait
        self._lost_cb = None
        
        self._timed_out = False
        self.is_acquired = False
        
    def register_lost_callback(self, cb):
        self._lost_cb = cb
        
    def release(self):
        if self.is_acquired:
            self._ac.cancel_all_goals()
            self.is_acquired = False
        
    def acquire(self, timeout):
        if self.is_acquired:
            return False
        
        self._timed_out = False
        if not self._ac.wait_for_server(rospy.Duration(timeout)):
            self._timed_out = True
            return False
        
        goal = AcquireRessourceGoal()
        self._ac.send_goal(goal, self._done_cb, self._active_cb)
        
        try:
            self._queue.get(True, timeout)
        except:
            self._timed_out = True
            return False
        
        return True
        
        
    def _done_cb(self, a, b):
        if self.is_acquired:
            self.is_acquired = False
            if self._lost_cb:
                t = threading.Thread(target=self._lost_cb)
                t.start()
        
    def _active_cb(self):
        if not self._timed_out:
            self.is_acquired = True
            self._queue.put(None)
        else:
            self._ac.cancel_goal()
        
