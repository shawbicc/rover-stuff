#!/usr/bin/env python3

import rospy
import os
import subprocess
import time
import signal

class JoystickManager:
    def __init__(self):
        self.joystick_processes = {}
        self.check_interval = 1.0  # Check every second
        self.allowed_joystick_name_nav = "USB Gamepad"  # joystick for nav
        self.allowed_joystick_name_arm = "Microntek              USB Joystick"  # joystick for arm

    def check_for_joysticks(self):
        joysticks = [js for js in os.listdir('/dev/input') if js.startswith('js')]
        for js in joysticks:
            if js not in self.joystick_processes:
                self.start_joy_node(js)
        
        # Check if any joysticks have been disconnected
        disconnected_joysticks = []
        for js in list(self.joystick_processes.keys()):
            if js not in joysticks:
                disconnected_joysticks.append(js)

        for js in disconnected_joysticks:
            self.stop_joy_node(js)

    def start_joy_node(self, js):
        device_path = os.path.join('/dev/input', js)
        try:
            with open('/sys/class/input/{}/device/name'.format(js), 'r') as f:
                joystick_name = f.read().strip()
            
            if joystick_name == self.allowed_joystick_name_nav:
                topic_name = '/nav/joy'
                # Generate a unique node name for each joystick
                node_name = 'joy_node_nav'
            
            elif joystick_name == self.allowed_joystick_name_arm:
                topic_name = '/arm/joy'
                # Generate a unique node name for each joystick
                node_name = 'joy_node_arm'
            
            else:
                rospy.logwarn("Joystick {} is not allowed. Skipping...".format(joystick_name))
                return

            # # Generate a unique node name for each joystick
            # node_name = 'joy_node_{}'.format(js)

            rospy.loginfo("Starting {} for {} on {}, publishing to {}".format(node_name, joystick_name, device_path, topic_name))

            # Use ROS remap argument to remap /joy topic to the specific joystick topic
            process = subprocess.Popen([
                'rosrun', 'joy', 'joy_node', 
                '__name:={}'.format(node_name),  # Set the node name
                '_dev:={}'.format(device_path), 
                'joy:={}'.format(topic_name)
            ])
            self.joystick_processes[js] = {'process': process, 'node_name': node_name}

        except Exception as e:
            rospy.logerr("Failed to start joy_node for {}: {}".format(js, e))

    def stop_joy_node(self, js):
        if js in self.joystick_processes:
            process_info = self.joystick_processes.pop(js)
            process = process_info['process']
            node_name = process_info['node_name']
            if process:
                rospy.loginfo("Stopping {} for {}".format(node_name, js))
                process.terminate()
                time.sleep(1)  # Allow some time for process to terminate
                if process.poll() is None:  # Check if process is still alive
                    process.kill()
                    rospy.loginfo("Forcefully killed {} for {}".format(node_name, js))

    def run(self):
        rospy.init_node('joystick_manager', anonymous=True)
        rate = rospy.Rate(1.0 / self.check_interval)

        while not rospy.is_shutdown():
            self.check_for_joysticks()
            rate.sleep()

if __name__ == '__main__':
    try:
        manager = JoystickManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass
