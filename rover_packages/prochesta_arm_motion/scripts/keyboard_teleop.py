#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import String

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


key_bindings_msg = """
Reading from the keyboard  and Publishing to a topic!
---------------------------
Press any key to publish it to the topic 'arm_cmd_to_hw_agent'
CTRL-C to quit
base -> b or B
shoulder -> s or S
elbow -> e or E
pitch -> p or P
roll -> r or R
grip -> g or G
press -> t or T
"""

class PublishThread(threading.Thread):
    def __init__(self):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('arm_cmd_to_hw_agent', String, queue_size = 1)
        self.key = ''
        self.condition = threading.Condition()
        self.done = False
        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, key):
        self.condition.acquire()
        self.key = key
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update('0')
        self.join()

    def run(self):
        msg = String()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait()

            # Copy state into message.
            msg.data = f"m{self.key}"

            self.condition.release()

            # Publish.
            self.publisher.publish(msg)

        # Publish empty message when thread exits.
        msg.data = f'm0'
        self.publisher.publish(msg)

class ForwardKinematics(object):
    def __init__(self) -> None:
        rospy.init_node('arm_keyboard_teleop')

        
    def getKey(self, settings, timeout):
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            rlist, _, _ = select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = '0'
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def saveTerminalSettings():
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)

    def restoreTerminalSettings(old_settings):
        if sys.platform == 'win32':
            return
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def run_fk(self):
        self.pub_thread = PublishThread()
        self.settings = ForwardKinematics.saveTerminalSettings()
        self.key_timeout = rospy.get_param("~key_timeout", 0.5)
        print(key_bindings_msg)
        while not rospy.is_shutdown():
            self.pub_thread.wait_for_subscribers()
            key = self.getKey(self.settings, self.key_timeout)
            if key == 'q' or key == '\x03':
                print("Quit request received. Stopping...")
                break
            self.pub_thread.update(key)
            
        self.pub_thread.stop()
        ForwardKinematics.restoreTerminalSettings(self.settings)
    

if __name__=="__main__":
    fk = ForwardKinematics()
    fk.run_fk()
