import threading
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

from colorama import Fore, Style
from pyjoystick.sdl2 import Key, Joystick, run_event_loop, stop_event_wait


joystick_binding_msg = """ 
Active button   L1 or R1 button (have to be pressed all the time to work)

Right joystick  up = shoulder down (looks like  forward)
                down = shoulder up (looks like  backward)

Left joystick   up = elbow up (looks like forward)
                down = elbow down (looks like backward)
                left = base ccw
                right = base cw

hat switches    up = pitch up
     (d-pad)    down = pitch down
                left = roll ccw
                right = roll cw

Gripper open    1
Gripper close   3
Press           2
"""
AXES = {"base": 0, "shoulder": 3, "elbow": 1, "pitch": 5, "roll": 4}

BUTTONS = {"gripper_open": 0, "gripper_close": 2, "press": 1}

GRIP_CLOSE = 1
GRIP_OPEN = 2

PRESS_ON = 1
PRESS_OFF = 2


def map_values(x, in_min, in_max, out_min, out_max):
    result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    # print(f"actual:{result}, abs:{abs(result)}")
    return int(result)


class PublishThread(threading.Thread):
    LAST_SPOKEN = 0

    def __init__(self):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher("arm_cmd_to_hw_agent", String, queue_size=1)
        self.text = ""
        self.condition = threading.Condition()
        self.done = False
        self.start()

    def wait_for_subscribers(self):
        i = -1
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print(
                    "Waiting for subscriber to connect to {}".format(
                        self.publisher.name
                    )
                )
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")
        if i != -1:
            print(f"{Fore.GREEN}Subscriber found{Style.RESET_ALL}")
            print(joystick_binding_msg)

    def update(self, text):
        self.condition.acquire()
        self.text = text
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update("0")
        self.join()

    def run(self):
        msg = String()
        rate = rospy.Rate(10)
        while not self.done or not rospy.is_shutdown():
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait()

            # Copy state into message.
            msg.data = f"m{self.text}"

            self.condition.release()

            # Publish.
            self.publisher.publish(msg)
            self.LAST_SPOKEN = rospy.Time().to_nsec()
            # rate.sleep()

        # Publish empty message when thread exits.
        msg.data = f"m0"
        self.publisher.publish(msg)


class ArmJoystick(object):
    JOYSTICK_MODEL = "Microntek              USB Joystick"
    ARM_JOYSTICK_ADDED = False
    ARM_TURBO = False
    ARM_ACTIVE = False
    L1_PRESSED = False
    R1_PRESSED = False
    scaling_factor = 0.5
    TURBO_SCALE = 1.0
    NORMAL_SCALE = 0.5
    IS_ALIVE = True

    def __init__(self) -> None:
        rospy.init_node("arm_forward_kinematics")
        self.joy_msg = None
        self.joy_node_sub = rospy.Subscriber(
            "/arm/joy", Joy, self.joy_callback, queue_size=1
        )
        self.pyjoystick_thread = threading.Thread(target=self.run_event)
        self.pyjoystick_thread.start()

    def run_event(self):
        run_event_loop(
            add_joystick=self.add_callback,
            remove_joystick=self.remove_callback,
            handle_key_event=self.key_received,
            alive=self.alive,
        )

    def alive(self):
        return self.IS_ALIVE

    def add_callback(self, joy: Joystick):
        print("Added, id:", joy.get_id())
        if joy.get_name().strip() == self.JOYSTICK_MODEL:
            self.ARM_JOYSTICK_ADDED = True
        print(f"ARM_JOYSTICK_ADDED:{self.ARM_JOYSTICK_ADDED}")

    def remove_callback(self, joy: Joystick):
        print("Removed", joy)
        if joy.get_name().strip() == self.JOYSTICK_MODEL:
            self.ARM_JOYSTICK_ADDED = False
        print(f"ARM_JOYSTICK_ADDED:{self.ARM_JOYSTICK_ADDED}")

    def key_received(self, key: Key):
        if key.joystick.get_name().strip() == self.JOYSTICK_MODEL:
            self.ARM_JOYSTICK_ADDED = True
        else:
            return
        if key == "Button 4":
            self.L1_PRESSED = key.value != 0
            self.ARM_ACTIVE = self.L1_PRESSED or self.R1_PRESSED
            self.scaling_factor = self.TURBO_SCALE if (self.L1_PRESSED and self.R1_PRESSED) else self.NORMAL_SCALE
            return
        if key == "Button 5":
            self.R1_PRESSED = key.value != 0
            self.ARM_ACTIVE = self.L1_PRESSED or self.R1_PRESSED
            self.scaling_factor = self.TURBO_SCALE if (self.L1_PRESSED and self.R1_PRESSED) else self.NORMAL_SCALE
            return

    def key_received_check(self):
        if not self.ARM_JOYSTICK_ADDED:
            return

        if self.ARM_ACTIVE:
            self.handle_when_active()

    # def handle_when_active(self):
    #     print("Handle when active called")

    def handle_when_active(self):
        # print(f"{Fore.GREEN}handling active:{key}{Style.RESET_ALL}", end="\r")
        if not self.joy_msg:
            return

        msg_text = "_"

        # Base
        msg_text += f"{map_values(self.joy_msg.axes[AXES['base']], -1.0, 1.0, -255 * self.scaling_factor, 255 * self.scaling_factor,)}_"

        # Shoulder
        msg_text += f"{map_values(self.joy_msg.axes[AXES['shoulder']], -1.0, 1.0, -255 * self.scaling_factor, 255 * self.scaling_factor)}_"

        # Elbow
        msg_text += f"{map_values(self.joy_msg.axes[AXES['elbow']], -1.0, 1.0, -255 * self.scaling_factor, 255 * self.scaling_factor)}_"

        # Yaw
        msg_text += "0_"  # Dummy

        # Pitch
        msg_text += f"{map_values(self.joy_msg.axes[AXES['pitch']], -1.0, 1.0, -255 * self.scaling_factor, 255 * self.scaling_factor)}_"

        # Roll
        msg_text += f"{map_values(self.joy_msg.axes[AXES['roll']], -1.0, 1.0, -255 * self.scaling_factor, 255 * self.scaling_factor)}_"

        # Gripper
        grip_command = "0_"
        if (
            self.joy_msg.buttons[BUTTONS["gripper_open"]] != 0
            and self.joy_msg.buttons[BUTTONS["gripper_close"]] == 0
        ):
            grip_command = f"{GRIP_OPEN}_"
        if (
            self.joy_msg.buttons[BUTTONS["gripper_close"]] != 0
            and self.joy_msg.buttons[BUTTONS["gripper_open"]] == 0
        ):
            grip_command = f"{GRIP_CLOSE}_"
        msg_text += grip_command

        # Press
        msg_text += (
            f"{PRESS_OFF if self.joy_msg.buttons[BUTTONS['press']] == 0 else PRESS_ON}_"
        )

        self.pub_thread.update(msg_text)
        pass

    def run_joy(self):
        self.pub_thread = PublishThread()
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            try:
                self.pub_thread.wait_for_subscribers()
            except Exception as e:
                break
            finally:
                self.IS_ALIVE = True
            if (not self.ARM_ACTIVE) or (
                rospy.Time().to_nsec() - self.pub_thread.LAST_SPOKEN > int(5e9)
            ):
                self.pub_thread.update("0")
                rate.sleep()
                continue
            if self.joy_msg:
                self.key_received_check()
                self.joy_msg = None
        self.stop()

    def stop(self):
        self.IS_ALIVE = False
        print("stop called")
        stop_event_wait()
        print(f"event stopped")
        self.pub_thread.stop()
        print(f"pub thread stopped")
        self.pyjoystick_thread.join()
        print(f"background thread stopped")

    def joy_callback(self, msg: Joy):
        self.joy_msg = msg
        # print(f"axes:{msg.axes}")
        # print(f"buttons:{msg.buttons}")
        # print(f"msg:{msg}")
        pass



if __name__ == "__main__":
    a = ArmJoystick()
    try:
        a.run_joy()
        print(f"{Fore.RED}hello{Style.RESET_ALL}")
        rospy.spin()
    except Exception as e:
        print(e)
    finally:
        print("calling stop")
        a.stop()
        print("Stopped")
