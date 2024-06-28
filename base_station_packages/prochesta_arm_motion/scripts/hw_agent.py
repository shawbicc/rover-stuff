import serial
from colorama import Fore, Style

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from prochesta_arm_motion.msg import SixJoints
from prochesta_arm_motion.srv import JointSrv, JointSrvResponse

# Settings for serial communication
PORT = "/dev/ttyACM0"  # Replace with your actual port name
BAUDRATE = 115200  # Adjust as needed
MSG_LEN = 45 # assuming 000.00_000.00_000.00_000.00_000.00_000.00\r\n # Although '\r' is 1 character not 2

GRIP_CLOSE = 1
GRIP_OPEN = 2

ROLL_CW = 700
ROLL_CCW = 800

PRESS_ON = 1
PRESS_OFF = 2

FLAG_ENCODER = 900
FLAG_MAGNET = 800

mag_MD = 0
mag_ML = 1
mag_MH = 2

JOINT_SENSOR_POSITIONS = []
PICO_CORE_TEMPERATURE = 0
LAST_UPDATED_ns = 0

class SerialCom(object):
    def __init__(self) -> None:
        super(SerialCom, self).__init__()

    def close_port(self):
        self.ser.close()

    def setup_serial(self):
        """Initializes serial communication with error handling."""
        rate_1 = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                self.ser = serial.Serial(PORT, BAUDRATE, timeout=1)
                rospy.loginfo(
                    f"{Fore.GREEN}Serial connection established!{Style.RESET_ALL}"
                )
                return self.ser
            except serial.SerialException as e:
                rospy.logerr(f"Serial connection failed: {e}")
                rospy.loginfo(f"{Fore.CYAN}Trying again{Style.RESET_ALL}")
                rate_1.sleep()
                # rospy.loginfo(f"{Fore.CYAN}Trying again in... 3{Style.RESET_ALL}")
                # rate_1.sleep()
                # rospy.loginfo(f"{Fore.CYAN}Trying again in... 2{Style.RESET_ALL}")
                # rate_1.sleep()
                # rospy.loginfo(f"{Fore.CYAN}Trying again in... 1{Style.RESET_ALL}")
                # rate_1.sleep()
                continue

    def send_data(self, data: str):
        try:
            if len(data) < MSG_LEN:
                data += '_'*(MSG_LEN - len(data) - 1) + '\n'
            rospy.loginfo(f"serial.write:'{data.encode('utf-8')}'")
            self.ser.write(data.encode("utf-8"))
            self.ser.flush()  # Ensure all data is sent
        except serial.SerialException as e:
            rospy.logerr(f"Error sending data: {e}")
            self.close_port()
            self.setup_serial()

    def _receive_data(self) -> str:
        self.ser.readline() # to clear up if any, from previous transfer
        raw_bytes = self.ser.read(MSG_LEN)
        if len(raw_bytes) != MSG_LEN:
            rospy.logwarn(f"Got malformed data, raw_bytes:{raw_bytes}")
            self._receive_data()
        # rospy.loginfo(f"{Fore.CYAN}raw_bytes:{raw_bytes}{Style.RESET_ALL}")
        # deco = raw_bytes.decode("utf-8")
        # rospy.loginfo(f"{Fore.CYAN}raw_bytes_decoded:{deco}{Style.RESET_ALL}")
        return raw_bytes.decode("utf-8")
    
    def receive_data(self) -> str:
        try:
            return self._receive_data()
        except serial.SerialException as e:
            rospy.logerr(f"Error receiving data: {e}")
            self.close_port()
            self.setup_serial()



serCom = SerialCom()
arm_notif_pub = None

def cb_from_controller(msg: SixJoints):
    # rospy.loginfo("in subscription callback:")
    # rospy.loginfo(msg)
    data = f"{msg.joint1:.2f}"
    data += "_" + f"{msg.joint2:.2f}"
    data += "_" + f"{msg.joint3:.2f}"
    data += "_" + f"{msg.joint4:.2f}"
    data += "_" + f"{msg.joint5:.2f}"
    data += "_" + f"{msg.joint6:.2f}"
    rospy.loginfo(f"{Fore.YELLOW}Got command..Sending serial:{data}{Style.RESET_ALL}")
    serCom.send_data(f"a{data}")  # Leading a for differenciating between position cmds and gripper/press cmds


def service_server(request):
    if request.req != 1.0:
        rospy.logerr("Who dafuq sent this request!!!???")
    # This means the message is older than 200millis which is twice the microcontrollers
    # loop delay so callback should wait for another msg to 
    if rospy.Time().to_nsec() - LAST_UPDATED_ns > int(2e8): # 200 millis
        rospy.Rate(10).sleep()
    # rospy.loginfo("got request...........")
    res = JointSrvResponse()
    for i, dt in enumerate(JOINT_SENSOR_POSITIONS):
        if (dt == FLAG_ENCODER or dt == FLAG_MAGNET + mag_MH or dt == FLAG_MAGNET + mag_ML):
            # arm_notif_pub.publish(f"{i}:{dt}")
            dt = 0 # for controller
        setattr(res, f"joint{i+1}", dt) # e.g. res.joint1 = float_data[0]
    # rospy.loginfo(f"float_data:{JOINT_SENSOR_POSITIONS}")
    return res

#This callback is for manual command,
#  only way to control gripper and optional way to override other joints
def cb_arm_cmd(msg: String):
    # rospy.loginfo(f"Got gripper or roll command:{msg.data}")
    serCom.send_data(msg.data)
        

def read_from_arduino_and_store(event):
    global JOINT_SENSOR_POSITIONS, LAST_UPDATED_ns
    data = serCom.receive_data()
    # rospy.loginfo(f"{Fore.CYAN}in_service_server:data from serial:{data}{Style.RESET_ALL}")
    data = data.rstrip('\n')
    data = data.rstrip('\r')
    data = data.rstrip('_')
    # rospy.loginfo(f"{Fore.GREEN}data after rstrip:{data}{Style.RESET_ALL}")
    try:
        float_data = [float(s) for s in data.split("_")]
    except Exception as e:
        rospy.logwarn(f"Got malformed data, error:{e}")
        if data.find("Emergency Stopped") > -1:
            # Emergency stop
            pass
        return # ignore faulty message
    else: # Message is well structured
        # rospy.loginfo(f"{Fore.GREEN}{float_data}{Style.RESET_ALL}")
        JOINT_SENSOR_POSITIONS = float_data[:-1] # without the last one
        PICO_CORE_TEMPERATURE = float_data[-1]
        LAST_UPDATED_ns = rospy.Time().to_nsec()
        notif_msg = Float64MultiArray()
        notif_msg.data = float_data
        arm_notif_pub.publish(notif_msg)

def main():
    try:
        global PORT
        global arm_notif_pub

        rospy.init_node('hw_agent')
        
        PORT = rospy.get_param(param_name="port", default=PORT)
        serCom.setup_serial()
        
        rospy.Subscriber('controller_to_hw_agent', SixJoints, cb_from_controller, queue_size=1)
        rospy.Service('hw_agent_to_controller', JointSrv, service_server)
        rospy.Subscriber('arm_cmd_to_hw_agent', String, cb_arm_cmd, queue_size=1)
        # This publisher is for interpreting different constants like
        # MAG_H and FLAG ENCODER and publishing fot the rest of the rosnodes
        arm_notif_pub = rospy.Publisher('arm_status_notify', Float64MultiArray, queue_size=1)
        keep_reading_timer = rospy.Timer(rospy.Duration(0.1), read_from_arduino_and_store)
        
        rospy.spin()
    except serial.SerialException as e:
        rospy.logerr(f"Error receiving data: {e}")
        serCom.close_port()
        serCom.setup_serial()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

    serCom.close_port()
